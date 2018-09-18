#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//size_t N = ; //see mpc_params below
//double dt = ;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

double ref_cte = 0;
double ref_epsi = 0;
//double ref_v = 100;  

/* MPC params is as follow
  * 0 ref_v
  * 1 cte error
  * 2 epsi error
  * 3 speed error
  * 4 delta  
  * 5 acceleration
  * 6 delta diff
  * 7 acceleration diff
  * 8 N
  * 9 dt
  * 10 cte in curves
  */
std::vector<double> mpc_params ={
  100,//118.667,//109.199,//107.054, //ref_v //if it works well, try to increase this!
  2500.29, //cte error
  1471,//1573.93,//2105.37,//2087.38, //epsi error
  2,//1.01897, //speed error
  8,//5.9, //delta  
  0,//2.60603,//4.97048, //acceleration
  186.957,//167.7,//218.9, //delta diff
  29.8539,//21.8939,//9.57787,//acceleration diff
  9, // N
  0.088,//dt
  0.055};//0.055}; 
//current PID: 	|118.667	|2205.05	|2481.32	|2.06295	|8.3395	|-0.70617	|386.957	|29.8539	|11	|0.0807932	|0.026	|467.2

std::vector<double> twiddleDP = {
  5, //ref_v
  100, //cte error
  100, //epsi error
  1, //speed error
  3, //delta  
  2, //acceleration
  50, //delta diff
  4, //acceleration diff
  5,
  0.05,
  0.001}; 

//This vector restricts decimals for twiddle, 
std::vector<double> twiddleDecimals = {
  2, //ref_v
  0, //cte error
  0, //epsi error
  4, //speed error
  4, //delta 
  2, //acceleration
  3, //delta diff
  5, //acceleration diff
  0,
  5,
  4};  


size_t N = int(mpc_params[7]);; //TODO: is it possible to move these into mpc-params?
double dt = mpc_params[8];;

double x_start = 0;
double y_start = x_start + N;
double psi_start = y_start + N;
double v_start = psi_start + N;
double cte_start = v_start + N;
double epsi_start = cte_start + N;
double delta_start = epsi_start + N;
double a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    fg[0] = 0; //Cost


    const AD<double> x = vars[v_start]*N*dt;
    const AD<double> last_y = coeffs[0]+
                    coeffs[1]*x+
                    coeffs[2]*x*x+
                    coeffs[3]*x*x*x;
    // last_y gives us how far in y direction we need to go.

    //Mostly copy-paste from lectures and/or the QA-video
    //Reference State Cost

    for(int i=0; i<N; i++){
      // last_y will here move the points thowards the inner part of the curve
      fg[0] += mpc_params[1] *CppAD::pow(vars[cte_start + i] + mpc_params[10]*last_y,2)
            +  mpc_params[2] *CppAD::pow(vars[epsi_start + i],2)
            +  mpc_params[3] *CppAD::pow(vars[v_start + i] - mpc_params[0], 2);

            
    }
    for (int i = 0; i<N-1; i++){
      fg[0] += mpc_params[4] *CppAD::pow(vars[delta_start + i],2)
            +  mpc_params[5] *CppAD::pow(vars[a_start + i], 2);
    }
    for(int i=0;i<N-2;i++){
      fg[0] += mpc_params[6] *CppAD::pow(vars[delta_start + i + 1] - vars[delta_start+i],2)
            +  mpc_params[7] *CppAD::pow(vars[a_start + i + 1] - vars[a_start + i],2);
    }

    //todo setup constraint

    fg[1+x_start] = vars[x_start];
    fg[1+y_start] = vars[y_start];
    fg[1+psi_start] = vars[psi_start];
    fg[1+v_start] = vars[v_start];
    fg[1+cte_start] = vars[cte_start];
    fg[1+epsi_start] = vars[epsi_start];

    // The rest of teh constrains
    for (int i = 0; i < N-1; i++){
      // at time t+1
      AD<double> x1 = vars[x_start + i +1];
      AD<double> y1 = vars[y_start + i +1];
      AD<double> psi1 = vars[psi_start + i +1];
      AD<double> v1 = vars[v_start + i +1];
      AD<double> cte1 = vars[cte_start + i +1];
      AD<double> epsi1 = vars[epsi_start + i +1];
      // at time t
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0 
                    + coeffs[3] * x0*x0*x0;
      AD<double> psides0 = CppAD::atan(3*coeffs[3] * x0*x0 
                    + 2*coeffs[2] * x0 + coeffs[1]);

      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);


    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  best_speed=0;
  counter=0;
}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  N=int(mpc_params[8]);
  dt=mpc_params[9];
  x_start = 0;
  y_start = x_start + N;
  psi_start = y_start + N;
  v_start = psi_start + N;
  cte_start = v_start + N;
  epsi_start = cte_start + N;
  delta_start = epsi_start + N;
  a_start = delta_start + N - 1;


  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;  
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  
  size_t n_vars = N * 6 + (N-1) *2;
  size_t n_constraints = N*6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  //set all non-actuators
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }
  //s
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332*Lf;
    vars_upperbound[i] =  0.436332*Lf;  
  }
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] =  1.0;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          1.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (int i = 0; i < N-1; i++){
    result.push_back(solution.x[x_start + i +1]);
    result.push_back(solution.x[y_start + i +1]);
  }

  return result;
}

bool MPC::twiddle(double speed,int x, int y, double cte){ 
    /*
    * This function will execute the Twiddle. 
    * 
    * This will be a bit different than in the pid.
    * each time we only go 1 lap around, we will then give the score as avrage speed.
    * we will thus only care about the speed, the cte is not directly important,
    * But if the cte increase too much, we will go of track anyway.
    * 
    */
  total_speed += speed;
  if(speed > max_speed){
    max_speed = speed;
  }
  counter++;
  //double dist = sqrt(pow(x-old_x,2)+pow(y-old_y,2));
 // old_x=x;
 // old_y=y;
  //cout << "distance  " << dist << endl;
  bool reset = false;
  if (fabs(cte) > 3 || (counter > 50 && speed < 5)){
    cout << "outside or stuck (cte=" << cte << " counter=" << counter << " speed=" << speed << "), so do a reset and";
    total_speed=0;
    reset = true;
  }
  if(reset || (x<0 && x>-30) && y>0 ){
    //here we do the twiddle
    double speed_score = (total_speed/counter + max_speed)/2;
    if (speed_score > best_speed ){
      if(best_times < 1){
        cout << "This was the best so far, but to not make a winner just for luck, let's try again!" << endl;
        best_times++;
      }else{
      cout << endl <<  endl <<"---------------------------------------------------------" << endl <<
              " - - - NEW BEST: " <<speed_score << " mph when setting param#" <<
              parameterNo << " to " << mpc_params[parameterNo] << endl << "Average speed: " << total_speed/counter << ". Max speed: " << max_speed << endl << 
              "---------------------------------------------------------" << endl << endl;
      best_speed = speed_score;
      twiddleDP[parameterNo] = ceil(twiddleDP[parameterNo]*1.1*pow(10,twiddleDecimals[parameterNo]))/pow(10,twiddleDecimals[parameterNo]);
      parameterNo = (parameterNo+1) % twiddleDP.size();
      mpc_params[parameterNo] += twiddleDP[parameterNo];
      twiddleCheckNeg = true;
      cout << "Now start twiddle the next parameter #: " << parameterNo  << endl;
        best_times=0;
      }

    }else{
      best_times=0;
      if(!reset) cout << "The average speed " << speed_score <<" in less than " <<best_speed << ", so let's" ;
      if(twiddleCheckNeg){
          cout << " try negative may" << endl;
          mpc_params[parameterNo] -= 2*twiddleDP[parameterNo];
          twiddleCheckNeg = false;
      }else{
          cout << " try lower DP for parameter "  << parameterNo;
          mpc_params[parameterNo] += twiddleDP[parameterNo];  
          twiddleDP[parameterNo]  = floor(twiddleDP[parameterNo]*0.9*pow(10,twiddleDecimals[parameterNo]))/pow(10,twiddleDecimals[parameterNo]);
          if(twiddleDP[parameterNo] < 1/pow(10,twiddleDecimals[parameterNo])){
            twiddleDP[parameterNo] = 1/pow(10,twiddleDecimals[parameterNo]);
          }
          parameterNo = (parameterNo+1) % twiddleDP.size();
          twiddleCheckNeg = true;
          cout << ", the next parameter #:" << parameterNo  << endl;
          mpc_params[parameterNo] += twiddleDP[parameterNo]; //Note, this is the next parameher
      }
    }
    //std::cout << std::fixed;
    //std::cout << std::setprecision(2);
    //Todo fix better output
    cout << "------------ \t|ref_v\t|cte\t|epsi\t|speed\t|delta\t|accel\t|de-d\t|acc-diff\t|N\t|dt" << endl;
    cout << "current PID: ";
    for(auto &k:mpc_params){
    cout << "\t|" << k ;
    }
    cout << endl << "current DP : ";
    for(auto &k:twiddleDP){
    cout << "\t|"<< k ;
    }
    cout << endl << endl;
    counter = 0;
    total_speed = 0;
    max_speed=0;
    return true;
  } 
  return false;
}