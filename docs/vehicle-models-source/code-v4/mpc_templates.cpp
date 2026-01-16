// mpc_templates.cpp
#include "mpc_templates.hpp"
#include <osqp.h>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>

namespace nhm {

static void discretizeBicycle(double L, double dt,
                              double x, double y, double yaw, double v, double delta,
                              Eigen::Matrix<double,5,5>& A, Eigen::Matrix<double,5,2>& B)
{
    // States: [x y psi v delta]'; Inputs: [a, ddelta]
    double c = std::cos(yaw), s = std::sin(yaw);
    double sec2 = 1.0/(std::cos(delta)*std::cos(delta));
    double tan_d = std::tan(delta);

    // Continuous-time Jacobians (rough small-angle linearization)
    A.setZero(); B.setZero();
    // xdot = v cos(psi)
    A(0,2) = -v*s; A(0,3) = c;
    // ydot = v sin(psi)
    A(1,2) =  v*c; A(1,3) = s;
    // psidot = v/L * tan(delta)
    A(2,3) = tan_d / L;                    // d(psi_dot)/dv
    A(2,4) = (v/L) * sec2;                 // d(psi_dot)/d(delta)
    // vdot = a
    // deltadot = ddelta

    // Discretize (Euler)
    A = Eigen::Matrix<double,5,5>::Identity() + A*dt;
    B(3,0) = dt;                            // v_{k+1}
    B(4,1) = dt;                            // delta_{k+1}
}

MpcSolution solveMpcOsqp(const MpcParams& P,
                         const std::vector<RefSample>& ref,
                         const Eigen::Vector4d& x0)
{
    // Build linear time-varying MPC with nx=5 (x,y,psi,v,delta), nu=2 (a, ddelta)
    const int nx=5, nu=2, N=P.N;
    if ((int)ref.size() < N+1) {
        // require N+1 reference states (including terminal)
        MpcSolution s; return s;
    }

    // Assemble A_k, B_k around reference (use delta_ref from kappa)
    std::vector<Eigen::Matrix<double,5,5>> A(N);
    std::vector<Eigen::Matrix<double,5,2>> B(N);

    auto kappa_to_delta = [&](double k){ return std::atan(P.L * k); };

    for (int k=0;k<N;++k){
        double x=ref[k].x, y=ref[k].y, yaw=ref[k].yaw, v=ref[k].v;
        double delta = kappa_to_delta(ref[k].kappa);
        discretizeBicycle(P.L, P.dt, x,y,yaw,v,delta, A[k], B[k]);
    }

    // Decision vector z = [x0..xN, u0..u_{N-1}]
    const int nxN = nx*(N+1);
    const int nuN = nu*N;
    const int nz = nxN + nuN;

    // Cost H, q
    // diag(Q.., QN, R..)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nz, nz);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(nz);

    Eigen::Matrix<double,5,5> Q = Eigen::Matrix<double,5,5>::Zero();
    // position
    Q(0,0) = P.w_pos; Q(1,1) = P.w_pos;
    Q(2,2) = P.w_yaw; Q(3,3) = P.w_v;
    Q(4,4) = 0.1; // small weight to keep delta near ref

    Eigen::Matrix<double,5,5> QN = Q * 2.0;

    Eigen::Matrix<double,2,2> R = Eigen::Matrix<double,2,2>::Zero();
    R(0,0) = P.w_u; R(1,1) = P.w_u;

    // Place state costs
    for (int k=0;k<=N;++k){
        const auto& rk = ref[k];
        Eigen::Matrix<double,5,1> xr; xr<<rk.x, rk.y, rk.yaw, rk.v, std::atan(P.L*rk.kappa);
        int idx = k*nx;
        const auto& Qk = (k==N)? QN : Q;
        H.block(idx, idx, nx, nx) += Qk;
        q.segment(idx, nx) -= Qk * xr;
    }
    // Input costs
    for (int k=0;k<N;++k){
        int idx = nxN + k*nu;
        H.block(idx, idx, nu, nu) += R;
    }

    // Dynamics + bounds in A,l,u
    // Equality dynamics: x_{k+1} = A_k x_k + B_k u_k
    // Initial condition: x0 = x_init (with delta initialized from ref[0])

    // Count constraints
    int n_dyn_eq = nx*N + nx; // includes x0 fix

    // Box bounds for states (per k) and inputs => as inequalities: l <= I z <= u

    // state bounds vectors per k
    auto inf = 1e20;
    std::vector<double> xl(nx), xu(nx);
    // x,y bounds
    xl[0]=P.x_min; xu[0]=P.x_max;
    xl[1]=P.y_min; xu[1]=P.y_max;
    // yaw unbounded
    xl[2]=-inf; xu[2]=inf;
    // v bounds (supports signed v for backing)
    xl[3]=P.v_min; xu[3]=P.v_max;
    // delta bounds
    xl[4]=P.delta_min; xu[4]=P.delta_max;

    // input bounds
    std::vector<double> ul(nu), uu(nu);
    ul[0]=P.a_min;    uu[0]=P.a_max;      // a
    ul[1]=-P.ddelta_max; uu[1]= P.ddelta_max; // ddelta

    // Sizes
    int n_state_bounds = nx*(N+1);
    int n_input_bounds = nu*N;
    int n_ineq = n_state_bounds + n_input_bounds;

    // Total constraints
    int n_con = n_dyn_eq + n_ineq;

    // Build A (sparse) in triplets
    struct Trip { int r,c; double v; };
    std::vector<Trip> trips;
    auto push_block = [&](int r0, int c0, const Eigen::MatrixXd& M){
        for (int i=0;i<M.rows();++i)
            for (int j=0;j<M.cols();++j){
                double val = M(i,j);
                if (std::abs(val)>1e-12) trips.push_back({r0+i, c0+j, val});
            }
    };

    Eigen::VectorXd l = Eigen::VectorXd::Zero(n_con);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_con);

    int row = 0;
    // Initial condition: x0 = x_init
    {
        Eigen::MatrixXd I0 = Eigen::MatrixXd::Zero(nx, nz);
        I0.block(0, 0, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
        push_block(row, 0, I0);
        Eigen::Matrix<double,5,1> x_init; x_init<<x0(0),x0(1),x0(2),x0(3), std::atan(P.L*ref[0].kappa);
        l.segment(row, nx) = x_init;
        u.segment(row, nx) = x_init;
        row += nx;
    }
    // Dynamics for k=0..N-1
    for (int k=0;k<N;++k){
        Eigen::MatrixXd blk = Eigen::MatrixXd::Zero(nx, nz);
        // I on x_{k+1}
        blk.block(0, (k+1)*nx, nx, nx) = Eigen::MatrixXd::Identity(nx,nx);
        // -A on x_k
        blk.block(0, k*nx, nx, nx) -= A[k];
        // -B on u_k
        blk.block(0, nxN + k*nu, nx, nu) -= B[k];
        push_block(row, 0, blk);
        l.segment(row, nx).setZero();
        u.segment(row, nx).setZero();
        row += nx;
    }

    // State bounds as I * z between xl/xu at each stage
    for (int k=0;k<=N;++k){
        Eigen::MatrixXd Izk = Eigen::MatrixXd::Zero(nx, nz);
        Izk.block(0, k*nx, nx, nx) = Eigen::MatrixXd::Identity(nx,nx);
        push_block(row, 0, Izk);
        for (int i=0;i<nx;++i){ l(row+i)=xl[i]; u(row+i)=xu[i]; }
        row += nx;
    }
    // Input bounds
    for (int k=0;k<N;++k){
        Eigen::MatrixXd Iuk = Eigen::MatrixXd::Zero(nu, nz);
        Iuk.block(0, nxN + k*nu, nu, nu) = Eigen::MatrixXd::Identity(nu,nu);
        push_block(row, 0, Iuk);
        for (int i=0;i<nu;++i){ l(row+i)=ul[i]; u(row+i)=uu[i]; }
        row += nu;
    }

    // Convert H,q,A,l,u to OSQP CSC
    // Build CSC for H (symmetric)
    auto to_csc = [&](const Eigen::MatrixXd& M){
        int m = M.rows(), n = M.cols();
        std::vector<int> Ap(n+1,0), Ai; std::vector<double> Ax;
        Ai.reserve(std::max(1, m*n/4));
        Ax.reserve(Ai.capacity());
        int nnz=0;
        for (int j=0;j<n;++j){
            for (int i=0;i<m;++i){
                double val = M(i,j);
                if (std::abs(val)>1e-12){ Ai.push_back(i); Ax.push_back(val); ++nnz; }
            }
            Ap[j+1]=nnz;
        }
        return std::tuple<std::vector<int>,std::vector<int>,std::vector<double>>(Ap,Ai,Ax);
    };

    // Symmetrize H minimally (upper triangle)
    Eigen::MatrixXd Pmat = 0.5*(H + H.transpose());

    // Build A from triplets (CSC)
    int m_con = n_con, n_var = nz;
    std::vector<int> A_col_ptr(n_var+1,0);
    std::vector<std::vector<std::pair<int,double>>> col(n_var);
    for (auto &t: trips) col[t.c].push_back({t.r,t.v});
    std::vector<int> A_row_idx; A_row_idx.reserve(trips.size());
    std::vector<double> A_val;  A_val.reserve(trips.size());
    int nnzA=0;
    for (int j=0;j<n_var;++j){
        A_col_ptr[j]=nnzA;
        auto &vec = col[j];
        std::sort(vec.begin(), vec.end(), [](auto&a,auto&b){return a.first<b.first;});
        for (auto &pr: vec){ A_row_idx.push_back(pr.first); A_val.push_back(pr.second); ++nnzA; }
    }
    A_col_ptr[n_var]=nnzA;

    // OSQP setup
    OSQPSettings *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData *data = (OSQPData *)c_malloc(sizeof(OSQPData));
    osqp_set_default_settings(settings);
    settings->verbose = 0;

    auto [Pp, Pi, Px] = to_csc(Pmat);

    data->n = n_var; data->m = m_con;
    data->P = csc_matrix(n_var, n_var, (OSQPInt)Px.size(), Px.data(), Pi.data(), Pp.data());
    data->q = const_cast<double*>(q.data());
    data->A = csc_matrix(m_con, n_var, (OSQPInt)A_val.size(), A_val.data(), A_row_idx.data(), A_col_ptr.data());
    data->l = const_cast<double*>(l.data());
    data->u = const_cast<double*>(u.data());

    OSQPWorkspace *work = osqp_setup(data, settings);
    osqp_solve(work);

    MpcSolution sol;
    if (work && work->solution && work->info->status_val == OSQP_SOLVED){
        Eigen::Map<Eigen::VectorXd> z(work->solution->x, n_var);
        sol.u_a.resize(N); sol.u_delta.resize(N);
        for (int k=0;k<N;++k){
            sol.u_a[k]     = z(nxN + k*nu + 0);
            sol.u_delta[k] = z(nxN + k*nu + 1);
        }
        sol.x.resize(N+1); sol.y.resize(N+1); sol.yaw.resize(N+1); sol.v.resize(N+1);
        for (int k=0;k<=N;++k){
            sol.x[k]   = z(k*nx + 0);
            sol.y[k]   = z(k*nx + 1);
            sol.yaw[k] = z(k*nx + 2);
            sol.v[k]   = z(k*nx + 3);
        }
    }

    osqp_cleanup(work);
    c_free(data->A); c_free(data->P); c_free(data); c_free(settings);
    return sol;
}

#ifdef HAS_CASADI
// Nonlinear MPC using CasADi (skeleton)
#include <casadi/casadi.hpp>
MpcSolution solveMpcCasadi(const MpcParams& P,
                           const std::vector<RefSample>& ref,
                           const Eigen::Vector4d& x0){
    using casadi::MX; using casadi::DM; using casadi::Function; using casadi::SX;
    const int nx=5, nu=2, N=P.N; // state: x y psi v delta
    auto kappa_to_delta = [&](double k){ return std::atan(P.L * k); };

    // Variables over horizon
    MX X = MX::sym("X", nx, N+1);
    MX U = MX::sym("U", nu, N);

    // Dynamics integrator (RK4)
    auto f = [&](MX x, MX u){
        MX psi = x(2), v=x(3), delta=x(4);
        MX xdot = MX::vertcat({ v*casadi::cos(psi),
                                v*casadi::sin(psi),
                                (v/P.L)*casadi::tan(delta),
                                u(0),
                                u(1) });
        return xdot;
    };

    MX cost = MX::zeros(1,1);
    std::vector<MX> g; // constraints

    // Weights
    DM Q = DM::zeros(nx,nx); Q(0,0)=P.w_pos; Q(1,1)=P.w_pos; Q(2,2)=P.w_yaw; Q(3,3)=P.w_v; Q(4,4)=0.1;
    DM R = DM::zeros(nu,nu); R(0,0)=P.w_u; R(1,1)=P.w_u;

    // Initial state
    DM xinit = DM::zeros(nx,1); xinit(0)=x0(0); xinit(1)=x0(1); xinit(2)=x0(2); xinit(3)=x0(3); xinit(4)=kappa_to_delta(ref[0].kappa);
    g.push_back( X( casadi::Slice(), 0 ) - xinit );

    // Loop
    for (int k=0;k<N;++k){
        DM xr = DM::zeros(nx,1); xr(0)=ref[k].x; xr(1)=ref[k].y; xr(2)=ref[k].yaw; xr(3)=ref[k].v; xr(4)=kappa_to_delta(ref[k].kappa);
        MX e = X(casadi::Slice(),k) - xr;
        cost += casadi::mtimes(casadi::mtimes(e.T(), Q), e) + casadi::mtimes(casadi::mtimes(U(casadi::Slice(),k).T(), R), U(casadi::Slice(),k));
        // RK4 step
        MX xk = X(casadi::Slice(),k);
        MX uk = U(casadi::Slice(),k);
        MX k1 = f(xk, uk);
        MX k2 = f(xk + 0.5*P.dt*k1, uk);
        MX k3 = f(xk + 0.5*P.dt*k2, uk);
        MX k4 = f(xk + P.dt*k3, uk);
        MX x_next = xk + (P.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        g.push_back( X(casadi::Slice(),k+1) - x_next );
    }
    // Terminal cost
    DM xrN = DM::zeros(nx,1); xrN(0)=ref[N].x; xrN(1)=ref[N].y; xrN(2)=ref[N].yaw; xrN(3)=ref[N].v; xrN(4)=kappa_to_delta(ref[N].kappa);
    MX eN = X(casadi::Slice(),N) - xrN; cost += 2.0*casadi::mtimes(casadi::mtimes(eN.T(), Q), eN);

    // Bounds
    std::vector<double> lbx, ubx, lbg, ubg;
    lbx.resize(nx*(N+1) + nu*N, -1e20); ubx.resize(nx*(N+1) + nu*N, 1e20);
    // state bounds
    for (int k=0;k<=N;++k){
        int off = k*nx;
        lbx[off+0] = -1e20; ubx[off+0]= 1e20; // x
        lbx[off+1] = -1e20; ubx[off+1]= 1e20; // y
        lbx[off+2] = -1e20; ubx[off+2]= 1e20; // psi
        lbx[off+3] = P.v_min; ubx[off+3]= P.v_max; // v
        lbx[off+4] = P.delta_min; ubx[off+4]= P.delta_max; // delta
    }
    // input bounds
    for (int k=0;k<N;++k){
        int off = nx*(N+1) + k*nu;
        lbx[off+0] = P.a_min;    ubx[off+0] = P.a_max;
        lbx[off+1] = -P.ddelta_max; ubx[off+1] = P.ddelta_max;
    }

    // pack variables
    MX Z = casadi::vertcat({casadi::reshape(X, nx*(N+1), 1), casadi::reshape(U, nu*N, 1)});
    MXDict nlp = { {"x", Z}, {"f", cost}, {"g", casadi::vertcat(g)} };
    auto solver = casadi::nlpsol("solver", "ipopt", nlp);

    // g bounds: equality (zeros)
    int ng = (int)g.size()*nx;
    lbg.assign(ng, 0.0); ubg.assign(ng, 0.0);

    std::map<std::string, casadi::DM> arg;
    arg["lbx"] = lbx; arg["ubx"] = ubx; arg["lbg"] = lbg; arg["ubg"] = ubg;

    auto res = solver(arg);
    DM z = res.at("x");
    std::vector<double> zv = std::vector<double>(z->begin(), z->end());

    MpcSolution sol; sol.u_a.resize(N); sol.u_delta.resize(N);
    sol.x.resize(N+1); sol.y.resize(N+1); sol.yaw.resize(N+1); sol.v.resize(N+1);
    for (int k=0;k<N;++k){
        int offu = nx*(N+1) + k*nu;
        sol.u_a[k]     = zv[offu+0];
        sol.u_delta[k] = zv[offu+1];
    }
    for (int k=0;k<=N;++k){
        int offx = k*nx;
        sol.x[k]   = zv[offx+0];
        sol.y[k]   = zv[offx+1];
        sol.yaw[k] = zv[offx+2];
        sol.v[k]   = zv[offx+3];
    }
    return sol;
}
#endif

} // namespace nhm
