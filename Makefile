runtest: test
	./test

test: test.c
	gcc test.c -o test

