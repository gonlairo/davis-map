#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <ostream>
#include <string>
#include <vector>

// we want to include csv.h

class CCSVWriter{
    protected:
    std::ostream &COutput;
    std::vector <std::string> RowOutput;

    public:
        CCSVWriter(std::ostream &ou);
        ~CCSVWriter();
        static std::string string_check(std::string str);
        bool WriteRow(const std::vector< std::string > &row);
};

#endif
