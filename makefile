
# Compiler
COMMON=-O2 -I../../include -L../../lib -pthread -Wl,-no-as-needed -Wl,-rpath,'$$ORIGIN'/../../lib
CC = gcc
CXX = g++

# Compile options - include paths for header files
CXXFLAGS = -std=c++11 
CFLAGS = -std=c11 

# header files directory
INCDIR = -I./include -I../../include -I/usr/include/eigen3
# -IC:/myproject/mujoco/mujoco-2.2.1-windows-x86_64/include

# Library paths
#LIBS = -lmujoco -lglfw -lm -lnlopt
LIBS = -lmujoco -lglfw -lnlopt

HEADDIR = include
SRCDIR = src
OBJDIR = obj

# List all source files
SRCS = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(SRCDIR)/*.c)

# Derive object file names from source file names
OBJS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(patsubst $(SRCDIR)/%.c, $(OBJDIR)/%.o, $(SRCS)))

# HEADERS = $(patsubst $(SRCDIR)/%.cpp, $(HEADDIR)/%.h, $(patsubst $(SRCDIR)/%.c, $(HEADDIR)/%.h, $(SRCS)))

# Root directory (i.e. current directory) of project
ROOT = main

# Linking
all: $(OBJDIR) $(OBJS)
	$(CXX) $(COMMON) $(CXXFLAGS) $(INCDIR) $(OBJS) $(LIBS) -o $(ROOT)

# Compile each source file to an object file
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(INCDIR) -c $< -o $@
	
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCDIR) -c $< -o $@

# Clean the project
clean:
	rm -f *.o $(ROOT)