#include <Controller.hpp>

PlayerClient playerRobot("localhost");
Position2dProxy p2dProxy(&playerRobot, 0);
LaserProxy laserProxy(&playerRobot, 0);

Robot r;

int main()
{
    laserProxy.RequestGeom();
	
    while(true)
    {
		r.updateState();
		
		r.printInfo();
        
        /*
         * TODO: remove this setVel and setRotVel!
         * experimental circle, just to see if everything is OK.
         */
		r.setVel(1);
		r.setRotVel(2);
		
        
		move(r.getVel(), r.getRotVel());
		sense();
        positionPredict();
	}
	
	return 0;
}

void move(double v, double w)
{
	r.setVel(v);
	r.setRotVel(w);
	p2dProxy.SetSpeed(v, w);
}

void sense()
{	
	r.updateLaserReadings();
	//r.printLaserReadings();
    
    /*
     * TODO: Write interpretation function, which will do some analysis
     * with laser readings and return distances to walls and corners, as
     * well as to the landmark, when found.
     * 
     * interpretMeasurements()
     */
}

void positionPredict()
{
    /*
     * TODO: Write functions to build matrices and vectors to be used
     * by the Kalman Filter, such as mean, covariance, control and mea-
     * surements vectors, always using armadillo linear algebra library.
     * 
     *  mean = createMean();
     *  cov = createCov();
     *  ut = createUt();
     *  zt = createZt();
     */
    
    kalmanFilter();
}

void kalmanFilter()
{
    /*
     * TODO: Implement Kalman Filter algorithm.
     * TODO: Write functions to create each Kalman Filter matrix, such
     * as At, Bt and Ct, using armadillo.
     */
}

void strategy()
{
    /*
     * TODO: Write this function, which will choose the best action to
     * take after analysing all avaiable data.
     */
}
