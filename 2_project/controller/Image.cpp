#include "Image.hpp"

Image::Image()
{
	// load an image  
	IplImage* init = cvLoadImage("project1_map.png");
	if(!init)
	{
		printf("Could not load image file: project1_map.png\n");
		exit(0);
	}
	
	// create a window
	cvNamedWindow("mainWindow", CV_WINDOW_AUTOSIZE); 
	cvMoveWindow("mainWindow", 1000, 00);
	
	// declare a destination IplImage object with correct size, depth and channels
	this->originalImage = cvCreateImage( cvSize((int)((init->width*500)/789) , (int)((init->height*500)/535) ), init->depth, init->nChannels );

	//use cvResize to resize source to a destination image
	cvResize(init, this->originalImage);

	// show the image
	particlesImage = cvCloneImage(originalImage);
	cvShowImage("mainWindow", this->particlesImage);
	
	//orange (255,165,0)
	//dark blue (25,25,112)
	// wait for a key
	cvWaitKey(100);
}

Image::~Image()
{
}

void Image::showParticlesPositions(Robot r, Landmark l1, Landmark l2)
{
	vector<particle> robotParticles;
	vector<particle> landmark1Particles;
	vector<particle> landmark2Particles;
	
	robotParticles = r.getParticles();
	landmark1Particles = l1.getParticles();
	landmark2Particles = l2.getParticles();
	
	if (robotParticles.size() == 0)
	{
		LOG(LEVEL_FATAL) << "Particle info";
		LOG(LEVEL_FATAL) << "Trying to access robot particles data while "
                         << "array size is zero.";
		
		return;
	}
	
	particlesImage = cvCloneImage(originalImage);
	
	unsigned int counter;
	for (counter = 0; counter < robotParticles.size(); counter++)
	{
		int yImage = int(((robotParticles[counter].x + 12.2)/24.4)*(470-42)) + 42;
		int xImage = int(((-robotParticles[counter].y + 12)/24)*(475-20)) + 20;
//		LOG(LEVEL_DEBUG) << "xImage = " << xImage << " e yImage = " << yImage;
		
		cvSet2D(this->particlesImage, xImage, yImage, CV_RGB(0,0,0));
		cvSet2D(this->particlesImage, xImage+1, yImage, CV_RGB(0,0,0));
		cvSet2D(this->particlesImage, xImage, yImage+1, CV_RGB(0,0,0));
		cvSet2D(this->particlesImage, xImage+1, yImage+1, CV_RGB(0,0,0));
	}
	
	
	for (counter = 0; counter < landmark1Particles.size(); counter++)
	{
		int yImage = int(((landmark1Particles[counter].x + 12.2)/24.4)*(470-42)) + 42;
		int xImage = int(((-landmark1Particles[counter].y + 12)/24)*(475-20)) + 20;
//		LOG(LEVEL_DEBUG) << "xImage = " << xImage << " e yImage = " << yImage;
		
		cvSet2D(this->particlesImage, xImage, yImage, CV_RGB(255,165,0));
		cvSet2D(this->particlesImage, xImage+1, yImage, CV_RGB(255,165,0));
		cvSet2D(this->particlesImage, xImage, yImage+1, CV_RGB(255,165,0));
		cvSet2D(this->particlesImage, xImage+1, yImage+1, CV_RGB(255,165,0));
	}
	
	
	for (counter = 0; counter < landmark2Particles.size(); counter++)
	{
		int yImage = int(((landmark2Particles[counter].x + 12.2)/24.4)*(470-42)) + 42;
		int xImage = int(((-landmark2Particles[counter].y + 12)/24)*(475-20)) + 20;
//		LOG(LEVEL_DEBUG) << "xImage = " << xImage << " e yImage = " << yImage;
		
		cvSet2D(this->particlesImage, xImage, yImage, CV_RGB(25,25,255));
		cvSet2D(this->particlesImage, xImage+1, yImage, CV_RGB(25,25,255));
		cvSet2D(this->particlesImage, xImage, yImage+1, CV_RGB(25,25,255));
		cvSet2D(this->particlesImage, xImage+1, yImage+1, CV_RGB(25,25,255));
	}
	
	// show the image
	cvShowImage("mainWindow", this->particlesImage);
	
	//orange (255,165,0)
	//dark blue (25,25,112)
	// wait for a key
	cvWaitKey(100);
}
