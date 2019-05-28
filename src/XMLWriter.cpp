#include <iostream>
#include <expat.h>
#include "XMLWriter.h"
#include "XMLEntity.h"
#include "StringUtils.h"
#include <istream>
#include <queue>


CXMLWriter::CXMLWriter(std::ostream &os) : XOuput(os)
{
}
CXMLWriter::~CXMLWriter()
{
    Flush();
}

bool CXMLWriter::Flush()
{
    if (XStack.empty())
    {
        return false;
    }
    
    while (XStack.empty())
    {
        WriteEntity(XStack.top());
        XStack.pop();
    }
    return true;   
}

std::string check_xml(std::string str)
{
    auto t1 = StringUtils::Replace(str, "&", "&amp;");
    auto t2 = StringUtils::Replace(t1, "<", "&lt;");
    auto t3 = StringUtils::Replace(t2, ">", "&gt;");
    auto t4 = StringUtils::Replace(t3, "\"", "&quot;");
    auto t5 = StringUtils::Replace(t4, "'", "&apos;");
    return t5;
}

bool CXMLWriter::WriteEntity(const SXMLEntity &entity)
{
    std::string temp;
    if (entity.DType == SXMLEntity::EType::StartElement)
    {
        temp += "<" + entity.DNameData;
        for (auto attr_pair : entity.DAttributes)
        {
            temp += " " + std::get<0>(attr_pair) + "=\"" + check_xml(std::get<1>(attr_pair)) + "\"";
        }

        temp += ">";
        XOuput << temp;
        XStack.push(entity);
        return true;
    }

    else if (entity.DType == SXMLEntity::EType::EndElement)
    {
        XOuput << "</" + entity.DNameData + ">";
        return true;
    }

    else if (entity.DType == SXMLEntity::EType::CharData)
    {
        XOuput << check_xml(entity.DNameData);
        return true;
    }
    else if (entity.DType == SXMLEntity::EType::CompleteElement)
    {
        std::string temp;
        temp += "<" + entity.DNameData;
        for (auto attr_pair : entity.DAttributes)
        {
            temp += " " + std::get<0>(attr_pair) + "=\"" + check_xml(std::get<1>(attr_pair)) + "\"";
        }

        temp += "/>";
        XOuput << temp;
        return true;
    }

    return false;
}