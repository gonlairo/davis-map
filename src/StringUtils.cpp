#include "../include/StringUtils.h"
#include <algorithm> 
#include <cctype>
#include <cstdio>

namespace StringUtils{
    
std::string Slice(const std::string &str, ssize_t start, ssize_t end){ //recieved guidance from Tutor Philip
    std::string s = ""; //empty string awaiting concatinization 

    if(start < 0){ //adds the length of the string to start if is is negative
        start += str.length();
    }

    if(end <= 0){ //adds the length of the string to end if negative or equals zero
        end += str.length();
    }
                           
    int index = 0; //variable to keep track of index number
    for (auto character : str) //iterates through each character in str
    {
    	if (index >= start && index < end) //checks if index is in the range between start and end
    	{
    		s += character; //adds character to s if in the correct range
    	}
    	index+=1; //moves index counter to next position 
    }
    return s; //returns the sliced string
}

std::string Capitalize(const std::string &str){
    
    std::string s = ""; //empty string

    s += toupper(str[0]); //capitalizes first character and adds it to new string

    for(int i = 1; i < str.length(); i++){ //ranges from second index to last
            s += tolower(str[i]); //makes character lower case and adds it to new string
        }

    return s;
}

std::string Title(const std::string &str){ //collaborted with Christopher Breed

    std::string s = "";

    bool isCap; //creates unassigned boolean value isCap

    for(auto character : str){ //iterates over str
        if(isCap == true){ //first letter of a word 
            s += toupper(character);
        }

        else{ //not the first letter of a word
            s += tolower(character);
        }

        isCap = false; //assumes we are not on a first letter of a word
        if(isalpha(character) == false){ //found a space, so need to capitalize the next letter
            isCap = true;
        }

    }

    s[0] = toupper(s[0]);
    return s;
}

std::string LStrip(const std::string &str){
    
    std::string s = str; //create copy of string

    int i = 0; //set i to first index
    while(i < str.length() && isspace(str[i])){ //while loop that counts up until a first char is reached
        i++;
    }
   
    return s.substr(i,str.length()-i); //creates a substring deleting white space on the left if it exists 
}

std::string RStrip(const std::string &str){
    
    std::string s = str; //create copy of string

    int i = str.length()-1; //set i to the length of the string - 1 to access the last index
    while(i >= 0 && isspace(str[i])){ //while loop that counts down until a char is reached
        i--;
    }
   
    return s.substr(0,i+1); //creates a substring from 0 to the the i+1 index
}

std::string Strip(const std::string &str){

    std::string s = str; //creates copy of string
    s = LStrip(s); //strips left white space
    s = RStrip(s); //strips right white space

    return s;
}

std::string Center(const std::string &str, int width, char fill){
    
    std::string s = str; //creates copy of str
    int x = str.length(); //finds the length of str
    int y = width - x; //finds the number of remaining characters
    int l = y/2; //divides remaining by 2 for left side
    int r = y - l; //alots the rest of the width to the right side

    std::string left; //left fill characters
    for(int i = 0; i < l; i++){
        left += fill;
    }

    std::string right; //right fill characters 
        for(int i = 0; i < r; i++){
            right += fill;
        }

    return left + s + right;

}

std::string LJust(const std::string &str, int width, char fill){
    std::string s = str; //creates copy of a string
    int x = str.length(); //finds the length of str
    int y = width - x; //finds the remaining width
    
    std::string left; //left fill characters
    for(int i = 0; i < y; i++){
        left += fill;
    }

    return s + left; //left justify
}

std::string RJust(const std::string &str, int width, char fill){
    std::string s = str; //creates copy of string
    int x = str.length(); //finds the length of the string
    int y = width - x; //finds the remaining width

    std::string right; //right fill characters
    for(int i = 0; i < y; i++){
        right += fill;
    }

    return right + s; //right justify
}

std::string Replace(const std::string &str, const std::string &old, const std::string &rep){
    
    std::string s; //create empty string

    for(int i = 0; i < str.length(); i++){ //interate over strinf
        if((str.substr(i, old.length()) == old)){ //if substring from index i to the length of old is the same as old
            s += rep; //add replacment to s
            i += old.length()-1; //move i to the next index after old
        }
        else{ //no replacemnt needed
            s += str[i];
        }
    }

    return s;
}

std::vector< std::string > Split(const std::string &str, const std::string &splt){ //guidance from Prof Nitta at office hours
    
   std::vector<std::string> splt_items; //creates an empty vector 
   std::string temp; //creates temp variable for substrings

   int s = 0; //start index tracker
   int e = 0; //end index tracker

   int i = 0; //keeps track of current index
   if(splt.empty()){ //deals with cases when no parameter is passed into splt
   		while(i < str.length()){ //loops until end of string is reached
   			if(isspace(str[i])){ //if a space is reached
   				splt_items.push_back(temp); //add temp to vector
   				temp.clear(); //clear temp
   				while(isspace(str[i])){ //move index forward until all white space is passed
   					i++;
   				}
   			}
   			else{ //index is not a white space
   				temp += str[i]; //add character in string to temp
   				i++; //move index forward
   				if(i == str.length()-1){ //for the last word in the string
   					temp += str[i]; //add last character to temp
   					splt_items.push_back(temp); //push temp to vector
   					i++; //finish the loop by moving index outside of the range
   				}
   			}
   		}
	}

	else{ //deals with cases when splt is a string; https://thispointer.com/how-to-split-a-string-in-c/ //used for splitting on a character

	   while((e = str.find(splt, s)) < str.length()){ //sets e to position of splt
	        temp = str.substr(s, e - s); //creates a substring from start until position of splt 
	        splt_items.push_back(temp); //adds substring to the vector 
	        s = e + splt.length(); //moves new start position to one index after the last occurance of splt
	    }

	    if(s < str.length()){ 
	        temp = str.substr(s);
	        splt_items.push_back(temp);
	    }
	}
	return splt_items;
}

std::string Join(const std::string &str, const std::vector< std::string > &vect){

    
    std::string s; //empty string to join elements of vect 
    for(int i = 0; i < vect.size(); i++){
        if(i == vect.size()-1){ //checks to see if last item in vect is reached
            s += vect[i];
        }
        else{
            s += vect[i] + str; //concatinates item and joining symbol
        }
    }
    return s;
}


std::string ExpandTabs(const std::string &str, int tabsize){
    std::string s; //empty string
    std::vector<std::string> new_vect;

    new_vect = Split(str, "\t"); //calls Split funciton to split str on \t
    int size_vect = new_vect.size();

    int i = 0;
    int x = 0;

    for(int index = 0; index < new_vect.size(); index++){ //traverse the vector of split items

    	i = new_vect[index].length(); //length of current item
    	if(tabsize == 0){ //prevents division by 0 error
    		s += new_vect[index];
    	}
    	else{ //tabsize is not 0
    		if(index == new_vect.size()-1){ //reached last item in new_vect so no white space is needed
    			s += new_vect[index];
    		}
    		else{
	    		x = tabsize - (i % tabsize); //finds the amount of white spaces needed following the item and before the next item
	    		std::string w;
	      		for(int y = 0; y < x; y++){ //creates a string of white space
	    			w += " ";
	    			}
	    		s += new_vect[index] + w; //adds white space following the item
    		}
    	}
    	
    }
    return s;
}

int EditDistance(const std::string &left, const std::string &right, bool ignorecase){ //https://en.wikibooks.org/wiki/Algorithm_Implementation/Strings/Levenshtein_distance#C++

    std::string LC = left; //copy of left string
    std::string RC = right; //copy of right string

    if (ignorecase == true) { //if true, make all lower

        for (std::string::size_type i = 0; i < LC.size(); i++) {
            LC[i] = tolower(LC[i]); //changes all of left to lower
        }

        for (std::string::size_type i = 0; i < RC.size(); i++) {
            RC[i] = tolower(RC[i]); //changes all of right string to lower
        }

    }

    int m = LC.size(); //length of left
    int n = RC.size(); //length of right

    std::vector<std::vector<int>> matrix(m + 1, std::vector<int>(n + 1)); //creates a vector within a vector

    matrix[0][0] = 0;

    for(int i = 1; i <= m; ++i){
        matrix[i][0] = i; 
    } 

    for(int i = 1; i <= n; ++i){
        matrix[0][i] = i;
    } 

    for (int i = 1; i <= m; ++i){

        for (int j = 1; j <= n; ++j){

        matrix[i][j] = std::min(std::min(matrix[i - 1][j] + 1, matrix[i][j - 1] + 1),
                            matrix[i - 1][j - 1] + (LC[i - 1] == RC[j - 1] ? 0 : 1)); //finds min of all three possible case (replace, erase, equal)

            }

        }

    return matrix[m][n];
}

}


