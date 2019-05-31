
#include "CSVReader.h"
#include <vector>
#include <queue>
#include <iostream>


//get rid of extra commas and quotes
CCSVReader::CCSVReader(std::istream &in) : CInput(in)
{
  // CInput(in)
  csv_init(&CParser, 0);
}

CCSVReader::~CCSVReader()
{
  csv_free(&CParser);
}

bool CCSVReader::End() const
{
  if(CInput.eof() && DataQueue.empty())
    {
      return true;
    }
  return false;
}



bool CCSVReader::ReadRow(std::vector< std::string > &row)
{
  while(DataQueue.empty() && !CInput.eof())
    {
      char Buffer[8192];
      CInput.read(Buffer,sizeof(Buffer));
      csv_parse(&CParser, Buffer, CInput.gcount(), CallBack1, CallBack2, this);

        if(CInput.eof())
        {
          csv_fini(&CParser, CallBack1, CallBack2, this);
        }
    }
    
    if(!DataQueue.empty())
    {
      row = DataQueue.front();
      DataQueue.pop();
      return true;
    }
    else
    {
      return false;
    }
}


void CCSVReader::CallBack1(void *str, size_t len, void *calldata)
{
  CCSVReader *Ptr = static_cast<CCSVReader*>(calldata);
  Ptr->RowInput.push_back(std::string((char*)str,len));
}

void CCSVReader::CallBack2(int ch, void *calldata)
{
  CCSVReader *Ptr = static_cast<CCSVReader *>(calldata);
  if ((ch == '\n') || (ch == '\r') || Ptr->CInput.eof())
  {
    Ptr->DataQueue.push(Ptr->RowInput);
    Ptr->RowInput.clear();
  }
}
