#include<stdio.h>
#include<unistd.h>
#include<stdlib.h>
#include<limits.h>
#include<string.h>
int main(int argc,char *argv[])
{
	FILE *finput,*foutput;
	char buffer[PIPE_BUF];
	int n;

	finput=popen("echo -e test!","r");  //输出重定向
	foutput=popen("cat","w");			//输入重定向

	read(fileno(finput),buffer,strlen("test!"));
	write(fileno(foutput),buffer,strlen("test"));
	
	pclose(finput);
	pclose(foutput);

	printf("\n");
	exit(EXIT_SUCCESS);
}
