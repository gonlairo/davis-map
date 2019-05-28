#ifndef STRINGUTILS_H
#define STRINGUTILS_H

#include <string>
#include <vector>

namespace StringUtils{
    
std::string Slice(const std::string &str, ssize_t start, ssize_t end=0);
std::string Capitalize(const std::string &str);
std::string Title(const std::string &str);
std::string LStrip(const std::string &str);
std::string RStrip(const std::string &str);
std::string Strip(const std::string &str);
std::string Center(const std::string &str, int width, char fill = ' ');
std::string LJust(const std::string &str, int width, char fill = ' ');
std::string RJust(const std::string &str, int width, char fill = ' ');
std::string Replace(const std::string &str, const std::string &old, const std::string &rep);
std::vector< std::string > Split(const std::string &str, const std::string &splt = "");
std::string Join(const std::string &str, const std::vector< std::string > &vect);
std::string ExpandTabs(const std::string &str, int tabsize = 4);
int EditDistance(const std::string &left, const std::string &right, bool ignorecase=false);

}

#endif
