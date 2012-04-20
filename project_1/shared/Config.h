/**
 * @file      Config.h
 * @author    George Andrew Brindeiro
 * @date      26/09/2010
 *
 * @attention Copyright (C) 2010
 * @attention UnBall Robot Soccer Team
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;

/**
 * @brief Configuração de módulos de software UnBall 
 *
 * Config é a classe responsável pelo armazenamento, leitura e gerenciamento 
 * de configurações para os diversos módulos de software da UnBall.
 *
 */
class Config
{
    public:
        Config() {};
        
        Config(string filename);
        
        ~Config() {};
        
        Config(const Config&);
        
        Config& operator=(const Config& other);

		// Reading Functions
		
		bool read(string filename);
		
		bool parseLine(string line);
		
		// Map functions
		
		bool empty();
		
		bool clear();
		
		int getSize();
		
		vector<string>& keys();
		
		// Vector functions
		
		int getSize(string key);
		
		string& get(string key);
		
		string& get(string key, int index);

		// Misc
		
		void print();
		
		bool printf(string filename);
		
		void printKeys();	

		string& name();

		map< string, vector <string> >::iterator& begin();
		
	 	map< string, vector <string> >::iterator& end();
		
    private:
		map< string, vector<string> > config;
		string objName;
		vector<string> cfgKeys;
		map< string, vector <string> >::iterator begin_iterator;
		map< string, vector <string> >::iterator end_iterator;
};

#endif // CONFIG_H
