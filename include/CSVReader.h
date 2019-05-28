#ifndef CSVREADER_H
#define CSVREADER_H

#include <istream>
#include <string>
#include <vector>
#include <queue>
#include <csv.h>

class CCSVReader{
    private:
        struct csv_parser CParser;
        std::istream &CInput; //reference is like a pointer but can only be assigned once and does not need to be deferenced
        std::vector<std::string> RowInput;
        

        static void CallBack1(void *str, size_t len, void *calldata);
        static void CallBack2(int ch, void *calldata);

    public:
        std::queue<std::vector<std::string>> DataQueue;
        CCSVReader(std::istream &in);
        ~CCSVReader();

        bool End() const;
        bool ReadRow(std::vector< std::string > &row);
};

#endif
