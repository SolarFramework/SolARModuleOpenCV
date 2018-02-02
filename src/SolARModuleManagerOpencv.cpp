/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARModuleManagerOpencv.h"

#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <fstream>
#include <regex>
#include <iostream>

using namespace std;

namespace xpcf  = org::bcom::xpcf;

namespace SolAR {
namespace MODULES {
namespace OPENCV {

SolARModuleManagerOpencv::SolARModuleManagerOpencv()
{

    // library loading
   m_xpcfComponentManager = xpcf::getComponentManagerInstance();;
    m_xpcfComponentManager->load() ;
    if (!m_xpcfComponentManager->isLoaded())
        loaded = false;
    else
        loaded = true;

}

SolARModuleManagerOpencv::SolARModuleManagerOpencv(const char *iniFile)
{
    // load xpcf info from ini file
    std::string xpcf_xmlPath, xpcf_libPath;

    // read ini file
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(iniFile, pt);
    #ifdef _DEBUG
    xpcf_xmlPath=pt.get<std::string>("OpenCV.xmlDebugPath");
    xpcf_libPath=pt.get<std::string>("OpenCV.libDebugPath");
    #else
    xpcf_xmlPath=pt.get<std::string>("OpenCV.xmlReleasePath");
    xpcf_libPath=pt.get<std::string>("OpenCV.libReleasePath");
    #endif

    // build a custom xml xpcf file based on config.ini informations
    std::ifstream filein(xpcf_xmlPath); // xpcf file to read from
    if(!filein)
    {
        LOG_ERROR("Error opening xpcf xml file!");
        loaded=false;
        return;
    }

    // only if lib path is specified in .ini file
    if (!xpcf_libPath.empty())
    {
        std::ofstream fileout("./xpcf_custom.xml"); // xpcf output file
        if(!fileout)
        {
            LOG_ERROR("Error opening xpcf xml file!");
            loaded=false;
            return;
        }
        std::string strTemp;
        std::string before="filepath=";
        std::string after=">";
        int i =1;
        while(std::getline(filein,strTemp))
        {
            if (i==3) // process third line, ie. the one containing the library path
            {
                string::size_type beg = strTemp.find(before);
                beg += before.size();
                string::size_type end = strTemp.find(after, beg);
                // initial lib path : strTemp.substr(beg, end-beg);
                std::string sub1,sub2;
                sub1=strTemp.substr(0,beg+1);
                sub2=strTemp.substr(end-1,strTemp.size()-1);
                strTemp=sub1+xpcf_libPath+sub2;
            }
                strTemp+="\n";
            ++i;
            fileout << strTemp;
        }
        fileout.close();
        // library loading based on custom xpcf xml file
        m_xpcfComponentManager = xpcf::getComponentManagerInstance();;
        m_xpcfComponentManager->load("./xpcf_custom.xml") ;
        if (!m_xpcfComponentManager->isLoaded())
            loaded = false;
        else
            loaded = true;

    }
    else  // load provided xpcf xml file
    {
        m_xpcfComponentManager = xpcf::getComponentManagerInstance();;
        m_xpcfComponentManager->load(xpcf_xmlPath.c_str()) ;
        if (!m_xpcfComponentManager->isLoaded())
            loaded = false;
        else
            loaded = true;
    }

}

}  // End namespace OPENCV
}  // End namespace MODULES
}  // End namespace SolAR

