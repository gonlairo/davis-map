#ifndef XMLREADER_H
#define XMLREADER_H

#include "XMLEntity.h"
#include <istream>
#include <expat.h>
#include <queue>
#include <vector>

class CXMLReader{
    
// change back: public to private
public: 
    std::istream &XInput;
    XML_Parser XParser;
    std::vector<SXMLEntity> DInput;
    std::queue < SXMLEntity> XQueue;
    static void startelement(void *userData, const XML_Char *name, const XML_Char **atts);
    static void endelement(void *userData, const XML_Char *name);
    static void chardata(void *userData, const XML_Char *name, int len);

    public : 
        CXMLReader(std::istream &is);
        ~CXMLReader();

    bool End();
    bool ReadEntity(SXMLEntity &entity, bool skipcdata = false);
    
};

#endif
