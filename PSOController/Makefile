cc	=	clang++
obj	=	PSOController
src	=	generateControl.cpp PSOCalcControl.cpp

all: $(src)
	$(cc) $(src) -o $(obj) -L. -lcontroller

clean:
	rm -f $(obj)
