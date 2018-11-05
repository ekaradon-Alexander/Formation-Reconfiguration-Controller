cc	:=	clang++
obj	:=	SQPController
src	:=	generateControl.cpp SQPCalcControl.cpp \
		Array.cpp QuadProg.cpp problem.cpp

all: $(src)
	$(cc) $(src) -o $(obj) -L. -lcontroller

clean:
	rm -f $(obj)
