#makefile
CXX = g++
CXXFLAGS = -std=c++14 $(INCLUDE) 
TESTFLAGS = -lgtest -lgtest_main -lpthread lib/libcsv.a

LIBCSVDIR = libcsv-3.0.3
LIBDIR = ./lib
LIBCSVNAME = libcsv.a

OBJDIR = ./obj
SRCDIR = ./src
INCLUDEDIR = ./include
INCLUDE = -I $(INCLUDEDIR)
CURDIR = $(shell pwd)

all: directories bin/findroute test $(LIBDIR)/$(LIBCSVNAME) 

test: testbin/testrouter
	./testbin/testrouter

bin/findroute: $(OBJDIR)/main.o $(OBJDIR)/CSVReader.o $(OBJDIR)/CSVWriter.o $(OBJDIR)/XMLReader.o $(OBJDIR)/XMLWriter.o $(OBJDIR)/StringUtils.o include/XMLEntity.h
	$(CXX) $(CXXFLAGS) $(OBJDIR)/main.o $(OBJDIR)/CSVReader.o $(OBJDIR)/CSVWriter.o $(OBJDIR)/XMLReader.o $(OBJDIR)/XMLWriter.o $(OBJDIR)/StringUtils.o -o bin/findroute -lexpat lib/libcsv.a


$(OBJDIR)/main.o: $(SRCDIR)/main.cpp  $(INCLUDEDIR)/MapRouter.h $(INCLUDEDIR)/CSVReader.h $(INCLUDEDIR)/CSVWriter.h $(INCLUDEDIR)/XMLReader.h $(INCLUDEDIR)/XMLWriter.h $(INCLUDEDIR)/XMLEntity.h $(INCLUDEDIR)/StringUtils.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/main.cpp -o $(OBJDIR)/main.o 

$(OBJDIR)/MapRouter.o: $(SRCDIR)/MapRouter.cpp $(INCLUDEDIR)/MapRouter.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/MapRouter.cpp -o $(OBJDIR)/MapRouter.o

$(OBJDIR)/CSVReader.o: $(SRCDIR)/CSVReader.cpp $(INCLUDEDIR)/CSVReader.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/CSVReader.cpp -o $(OBJDIR)/CSVReader.o

$(OBJDIR)/CSVWriter.o: $(SRCDIR)/CSVWriter.cpp $(INCLUDEDIR)/CSVWriter.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/CSVWriter.cpp -o $(OBJDIR)/CSVWriter.o

$(OBJDIR)/XMLReader.o: $(SRCDIR)/XMLReader.cpp $(INCLUDEDIR)/XMLReader.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/XMLReader.cpp -o $(OBJDIR)/XMLReader.o

$(OBJDIR)/XMLWriter.o: $(SRCDIR)/XMLWriter.cpp $(INCLUDEDIR)/XMLWriter.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/XMLWriter.cpp -o $(OBJDIR)/XMLWriter.o

$(OBJDIR)/StringUtils.o: $(SRCDIR)/StringUtils.cpp $(INCLUDEDIR)/StringUtils.h
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/StringUtils.cpp -o $(OBJDIR)/StringUtils.o

# libcsv
$(LIBCSVDIR)/Makefile:
	cd $(LIBCSVDIR); ./configure --prefix= $(CURDIR); cd ..

$(LIBCSVDIR)/libcsv.la: $(LIBCSVDIR)/Makefile
	cd $(LIBCSVDIR); make; cd ..

$(LIBDIR)/ $(LIBCSVNAME): $(LIBCSVDIR)/libcsv.la
	cd $(LIBCSVDIR); make install; cd ..ø


# tests
testbin/testrouter: $(OBJDIR)/CSVReader.o $(OBJDIR)/CSVWriter.o $(OBJDIR)/MapRouter.o $(OBJDIR)/testrouter.o $(INCLUDEDIR)/StringUtils.h 
	$(CXX) $(CXXFLAGS) $(OBJDIR)/CSVReader.o $(OBJDIR)/CSVWriter.o  $(OBJDIR)/MapRouter.o $(OBJDIR)/testrouter.o $(OBJDIR)/StringUtils.o -o testbin/testrouter $(TESTFLAGS) 

$(OBJDIR)/testrouter.o: $(SRCDIR)/testrouter.cpp $(OBJDIR)/StringUtils.o
	$(CXX) $(CXXFLAGS) -c $(SRCDIR)/testrouter.cpp -o $(OBJDIR)/testrouter.o


# directories
directories: obj/ testbin/ bin/

obj/:
	mkdir obj

testbin/:
	mkdir testbin

bin/:
	mkdir bin

clean:
	#rm -f include/lib.csv
	rm -f obj/*
	#rm -f lib/*
	rm -f testbin/*
	cd $(LIBCSVDIR); make clean;
	rm -f $(LIBCSVDIR)/Makefile
