#include <iostream>
#include <expat.h>
#include "XMLReader.h"
#include "XMLEntity.h"
#include <istream>
#include <queue>

void print_queue(std::queue<SXMLEntity> queue)
{
    std::cout << "**START QUEUE**" << std::endl;
    while (!queue.empty())
    {
        for (auto attr : queue.front().DAttributes)
        {
            std::cout << std::get<0>(attr) << "=" << std::get<1>(attr) << " / " <<  queue.front().DNameData << std::endl;
        }
        queue.pop();
    }
    std::cout << "**END QUEUE**" << std::endl;
}

CXMLReader::CXMLReader(std::istream &in) : XInput(in)
{
    XParser = XML_ParserCreate(NULL);
    XML_SetUserData(XParser, this);
    XML_SetElementHandler(XParser, startelement, endelement);
    XML_SetCharacterDataHandler(XParser, chardata);
}

CXMLReader::~CXMLReader()
{
    XML_ParserFree(XParser);
}


bool CXMLReader::ReadEntity(SXMLEntity &entity, bool skipcdata)
{
    while(skipcdata == true && !XQueue.empty() && XQueue.front().DType == SXMLEntity::EType::CharData)
    {
        XQueue.pop();
    }

    while (XQueue.empty() && !XInput.eof())
    {
        char Buffer[8192];
        XInput.read(Buffer, sizeof(Buffer));
        XML_Parse(XParser, Buffer, XInput.gcount(), XInput.eof());
    }

    if (!XQueue.empty())
    {
        // bool x = XQueue.front().DType == SXMLEntity::EType::CharData;
        //std::cout << x <<  std::endl;
        while(skipcdata == true && !XQueue.empty() && XQueue.front().DType == SXMLEntity::EType::CharData)
        {
            //std::cout << "pop" << std::endl;
            XQueue.pop();

        }

        entity = XQueue.front();
        XQueue.pop();
        return true;
    }
    else
    {
        return false;
    }
}

void CXMLReader::startelement(void *userData, const XML_Char *name, const XML_Char **atts)
{
    SXMLEntity entity;
    CXMLReader *ptr = static_cast<CXMLReader*>(userData);
    entity.DNameData = name;
    entity.DType = SXMLEntity::EType::StartElement;

    int index = 0;

    while (atts[index])
    {
        entity.SetAttribute(std::string(atts[index]), std::string(atts[index + 1]));
        index += 2;
    }

    ptr->XQueue.push(entity);
}

void CXMLReader::endelement(void *userData, const XML_Char *name)
{
    SXMLEntity entity;
    CXMLReader *ptr = static_cast<CXMLReader *>(userData);
    entity.DType = SXMLEntity::EType::EndElement;
    entity.DNameData = name;
    ptr->XQueue.push(entity);
}

void CXMLReader::chardata(void *userData, const XML_Char *name, int len)
{
    SXMLEntity entity;
    CXMLReader *ptr = static_cast<CXMLReader *>(userData);
    entity.DType = SXMLEntity::EType::CharData;
    entity.DNameData = std::string(name, len);
    ptr->XQueue.push(entity);
}

bool CXMLReader::End()
{
    if (XInput.eof() && XQueue.empty())
    {
        return true;
    }
    return false;
}

