all: MsgRcvr

MsgRcvr: MsgRcvr.o
	g++ MsgRcvr.o -o MsgRcvr

MsgRcvr.o: MsgRcvr.c
	g++ -c MsgRcvr.c

clean:
	rm -rf *o MsgRcvr
