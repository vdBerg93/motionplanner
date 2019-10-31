using namespace std;
// Matrices defined as [row][column]
struct sigmaX_type{
    double r,c;
    double m[4][4];
    sigmaX_type(): r(4), c(4){}
};
struct sigmaZ_type{
    double r,c;
    double m[2][2];
    sigmaZ_type(): r(2), c(2){}
};
struct mu_type{
    double r,c;
    double m[4];
    mu_type(): r(4), c(1){}
};
struct z_type{
    double r,c;
    double m[4];
    z_type():r(4), c(1){}
};
struct H_type{
    double r,c;
    double m[2][4];
    H_type(): r(2), c(4){}
};
struct F_type{
    double r,c;
    double m[4][4];
    F_type(): r(4), c(4){}
};

class KalmanFilter{
    private:
        vector<mu_type> mu_preds;           // Mean prediction
        vector<mu_type> mu_upds;            // Mean update
        vector<sigmaX_type> sigma_preds;     // Covariance prediction
        vector<sigmaX_type> sigma_upds;      // Covariance update
        vector<double> Ts;                  // Timestamps
        H_type H;
        F_type F;
        sigmaX_type sigmaX;
        sigmaZ_type sigmaZ;
        double sigma_Z[2][2];
        double Dx;
        double dt;
    public:
        KalmanFilter(mu_type mu, sigma_type sigma);
        void predictObs();
        void predictStep();
};

KalmanFilter::KalmanFilter(mu_type mu, sigma_type sigma){
    mu_preds.push_back(mu);             // Initialize
    sigma_preds.push_back(sigma);       // Initialize
    mu_upds.push_back(mu);              // Initialize
    sigma_upds.push_back(sigma);        // Initialize
    Ts.push_back(0);     // Initial timestamp
    Dx = 4;                             // the dimensionality of the state vector
    dt = 0.05;

    // x_t+1 = H x_t + epsilon_t, where noise epsilon_t ~ N(0, Sigma)
    // z_t   = F x_t + eta_t,     where noise eta_t ~ N(0, R)
    double noise_var_x_pos = 1e-1; // variance of spatial process noise
    double noise_var_x_vel = 1e-2; // variance of velocity process noise
    double noise_var_z = 3; // variance of measurement noise for z_x and z_y

    // define the linear dynamics
    //  F = NxN transition matrix
    //          [1 0 dt 0 ; 
    //          0 1 0 dt; 
    //          0 0 1 0; 
    //          0 0 0 1];                                       
    F.m[0][0] = 1; F.m[0][1] = 0; F.m[0][2] = dt; F.m[0][3] =  0;
    F.m[1][0] = 0; F.m[1][1] = 1; F.m[1][2] = 0; F.m[1][3] =  dt;
    F.m[2][0] = 0; F.m[2][1] = 0; F.m[2][2] = 1; F.m[2][3] =  0;
    F.m[3][0] = 0; F.m[3][1] = 0; F.m[3][2] = 0; F.m[3][3] =  1;
    //  H = MxN observation matrix
    //      [1 0 0 0; 
    //       0 1 0 0];     
    H.m[0][0] = 1; H.m[0][1] = 0; H.m[0][2] = 0; H.m[0][3] =  0;
    H.m[1][0] = 0; H.m[1][1] = 1; H.m[1][2] = 0; H.m[1][3] =  0;                                              
    // kf.Sigma_x=diag([noise_var_x_pos noise_var_x_pos noise_var_x_vel noise_var_x_vel]); //   = NXN covariance matrix
    sigmaX.m[0][0] = noise_var_x_pos;   sigmaX.m[0][1] = 0;                     sigmaX.m[0][2] = 0;                 sigmaX.m[0][3] =  0;
    sigmaX.m[1][0] = 0;                 sigmaX.m[1][1] = noise_var_x_pos;       sigmaX.m[1][2] = 0;                 sigmaX.m[1][3] =  0;
    sigmaX.m[2][0] = 0;                 sigmaX.m[2][1] = 0;                     sigmaX.m[2][2] = noise_var_x_vel;   sigmaX.m[2][3] =  0;
    sigmaX.m[3][0] = 0;                 sigmaX.m[3][1] = 0;                     sigmaX.m[3][2] = 0;                 sigmaX.m[3][3] =  noise_var_x_vel;
    // kf.Sigma_z=diag([noise_var_z noise_var_z]);                                         //   = MxM covariance matrix
    sigmaZ.m[0][0] = noise_var_z;   sigmaZ.m[0][1] = 0;
    sigmaZ.m[1][0] = 0;             sigmaZ.m[1][1] = noise_var_z;
}

void KalmanFilter::predictObs(){
    mu_type mu = mu_preds.back();
    sigmaX_type sigma = sigma_preds.back();
    ROS_WARN_ONCE("Implement matrtix multiplication");
    double z_mu[2];
    z_mu[0] = H.m[0][0]*mu.m[0] + H.m[0][1]*mu.m[1] + H.m[0][2]*mu.m[2] + H.m[0][3]*mu.m[3];
    z_mu[1] = H.m[1][0]*mu.m[0] + H.m[1][1]*mu.m[1] + H.m[1][2]*mu.m[2] + H.m[1][3]*mu.m[3];
    // z_mu = kf.H * mu;
    // z_Sigma = kf.H * Sigma * kf.H' + kf.Sigma_z;
}

void KalmanFilter::predictStep(){
    // get the last (i.e. updated) state from the previous time step
    mu_type mu_prev = mu_upds.back();
    sigma_type sigma_prev = sigma_upds.back();

    // Write here the two Kalman predict equations here to compute
    //   from previous state distribution of t-1, N(x_{t-1} |mu_prev, Sigma_prev)
    //   the predicted state distribution for t, N(x_t | mu, Sigma)
    // ----------------------
    ROS_WARN_STREAM("Implement matrix multiplication");
    // mu=kf.F*mu_prev;                            //predicted mean
    // Sigma=kf.F*Sigma_prev*(kf.F)'+kf.Sigma_x;   //predicted covariance

    // ----------------------

    // after the predicted mu and Sigma have been computed,
    //   store them in the struct as the latest predicted state
    mu_preds.push_back(mu);
    sigma_preds.push_back(sigma);
    // we also set the 'updated' values for the new time step
    // equal to the predict ones 
    mu_upds.push_back(mu);
    sigma_upds.push_back(mu);
    Ts.push_back(Ts.back()+dt);
}

void KalmanFilter::updateStep(z){
    // To compute the inverse of a matrix S,
    //  you can use `inv(S)`.
    // Use the `eye` function to create the identity matrix I

    // get the lastest predicted state, which should be the current time step
    mu_type mu = mu_upds.back();
    sigma_type sigma = sigma_upds.back();

    // Kalman Update equations go here
    //
    //   Use mu and Sigma to compute the 'updated' distribution decribed by
    //    mu_upd, Sigma_upd using the Kalman Update equations

    // et=z-kf.H*mu;                           //innovation(residual error)
    // St= kf.H*Sigma*kf.H' + kf.Sigma_z;      //innovation covariance
    // Kt=Sigma*kf.H'*inv(St);                 //Kalman gain
    // mu_upd=mu+Kt*et;                        //updated mean
    // Sigma_upd=(eye(kf.Dx)-Kt*kf.H)*Sigma;   //updated covariance

    // ok, store the result
    mu_upds.push_back(mu_upd);
    sigma_upds.push_back(sigma_upd);
}