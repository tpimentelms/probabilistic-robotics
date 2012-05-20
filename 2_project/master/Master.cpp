/**
 * @file      Master.cpp
 * @author    Matheus Vieira Portela
 * @date      18/04/2012
 */

#include <Master.h>
#include <Config.h>
#include <ProcessLogger.h>
#include <unistd.h>

int main(int argc, char* argv[])
{
	string command; // Hold command line
	
	// Get current directory
	char cwd[256];
	getcwd(cwd, 255);
	LOG(LEVEL_DEBUG) << "Current Path: " << cwd << endl;
	
	// Start simulation
	string simulation_path = cwd + string("/../simulation/");
	LOG(LEVEL_DEBUG) << "Simulation path: " << simulation_path;
	command = string("gnome-terminal --working-directory=") + simulation_path + string(" -x bash -c \"player project1.cfg; bash\"");
	
	LOG(LEVEL_INFO) << command.c_str();
	system(command.c_str());
	
	sleep(1);
	
	// Start controller
	string controller_path = cwd + string("/../controller/");
	LOG(LEVEL_DEBUG) << "Controller path: " << controller_path;
	command = string("gnome-terminal --working-directory=") + controller_path + string(" -x bash -c \"./Controller; bash\"");
	
	LOG(LEVEL_INFO) << command.c_str();
	system(command.c_str());
}
