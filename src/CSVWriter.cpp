#include "CSVWriter.h"
#include "StringUtils.h"
#include<vector>
#include<queue>
#include <iostream>
#include <fstream>

CCSVWriter::CCSVWriter(std::ostream &ou) : COutput(ou)
{
}

CCSVWriter::~CCSVWriter()
{
}

std::string CCSVWriter::string_check(std::string str)
{
  std::string ret = "\"";
  for (auto ch : str)
  {
    if (ch == '\"')
    {
      ret += ch;
      ret += "\"";
    }
    else
    {
      ret += ch;
    }
  }

  ret += "\"";
  //std::cout << "final: " << ret << std::endl;
  return ret;
}

bool CCSVWriter::WriteRow(const std::vector< std::string > &row)
{
  if (!row.empty())
  {
    for(auto field : row)
    {
      RowOutput.push_back(string_check(field));
    }


    COutput << StringUtils::Join(",", RowOutput) << std::endl;
    RowOutput.clear();
    return true;
  }
  else
  {
    return false;
  }
}
