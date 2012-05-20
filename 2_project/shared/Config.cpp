/**
 * @file      Config.cpp
 * @author    George Andrew Brindeiro
 * @date      26/09/2010
 *
 * @attention Copyright (C) 2010
 * @attention UnBall Robot Soccer Team
 */

#include <Config.h>
#include <ProcessLogger.h>

Config::Config(string filename)
{
	Config();
	
	if(!read(filename))
		LOG(LEVEL_ERROR) << "Could not open file " << filename;
	else
	{
		objName = filename;		
	}
}

Config::Config(const Config& cp)
{
	/*cp.config = config;
	cp.objName = objName;
	cp.begin_iterator = begin_iterator;
	cp.end_iterator = end_iterator;*/	
}

Config& Config::operator=(const Config& other)
{
	if (this != &other) // protect against invalid self-assignment
	{
		// 1: allocate new memory and copy the elements
		//int * new_array = new int[other.count];
		//std::copy(other.array, other.array + other.count, new_array);
			
		// 2: deallocate old memory
		//delete [] array;

		// 3: assign the new memory to the object
		//array = new_array;
		//count = other.count;
	}

	return *this; // by convention, always return *this
}

bool Config::read(string filename)
{
	ifstream cfg(filename.c_str(), ifstream::in);

	if(cfg.fail())
		return false;

	string line;

	while(cfg.good())
	{
		getline(cfg, line);
		parseLine(line);
	}

	cfg.close();

	return true;
}

bool Config::parseLine(string line)
{
	if((line[0] == '#') || (line.size() == 0))
		return false;

	// construct a stream from the string
	stringstream ss(line);

	// use stream iterators to copy the stream to the vector as whitespace separated strings
	istream_iterator<string> it(ss);
	istream_iterator<string> end;
	vector<string> tokens(it, end);

	LOG(LEVEL_VERBOSE) << "tokens size: " << tokens.size();

	string key(tokens[0]);
	tokens.erase(tokens.begin());
	config[key] = tokens;

	LOG(LEVEL_VERBOSE) << "config[ " << key
					   << "] size: " << config[key].size() << endl;

	return true;
}

bool Config::empty()
{
	return config.empty();
}

bool Config::clear()
{
	config.clear();
	
	return (config.size() == 0 ? true : false);
}

int Config::getSize()
{
	return config.size();
}

vector<string>& Config::keys()
{
	cfgKeys.clear();
	
	for(map< string, vector <string> >::iterator im = config.begin(); im != config.end(); im++)
	{
		cfgKeys.push_back(im->first);
	}

	return cfgKeys;
}

int Config::getSize(string key)
{
	return config[key].size();
}

string& Config::get(string key)
{
	return config[key][0];	
}

string& Config::get(string key, int index)
{
	return config[key][index];	
}

void Config::print()
{
	if(empty())
	{
		cout << "Config is empty!" << endl;
	}
	else
	{
		cout << "# UnBall Robot Soccer Team" << endl;
		cout << "# " << name() << " Configuration" << endl;
		cout << endl;
		
		for(map< string, vector <string> >::iterator im = config.begin(); im != config.end(); im++)
		{
			cout << im->first;
			
			for(vector<string>::iterator iv = im->second.begin(); iv != im->second.end(); iv++)
				cout << " " << *iv;
			
			cout << endl;
		}
	}
	
	cout << endl;
}

bool Config::printf(string filename)
{
	ofstream cfg(filename.c_str(), ofstream::out);
	
	if(cfg.fail())
		return false;

	cfg << "# UnBall Robot Soccer Team" << endl;
	cfg << "# " << name() << " Configuration" << endl;
	cfg << endl;
	
	for(map< string, vector <string> >::iterator im = config.begin(); im != config.end(); im++)
	{
		cfg << im->first;
		
		for(vector<string>::iterator iv = im->second.begin(); iv != im->second.end(); iv++)
			cfg << " " << *iv;
		
		cfg << endl;
	}

	cfg << endl;	

	cfg.close();

	return true;
}

void Config::printKeys()
{
	cout << "# UnBall Robot Soccer Team" << endl;
	cout << "# " << name() << " Keys" << endl;
	cout << endl;
	
	keys();
	
	for(vector <string>::iterator key = cfgKeys.begin(); key != cfgKeys.end(); key++)
	{
		cout << *key << endl;
	}
	
	cout << endl;
}

string& Config::name()
{
	return objName;
}

map< string, vector <string> >::iterator& Config::begin()
{
	begin_iterator = config.begin();
	return begin_iterator;
}

map< string, vector <string> >::iterator& Config::end()
{
	end_iterator = config.end();
	return end_iterator;
}
