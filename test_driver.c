#include <stdio.h>
#include <stdlib.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

static char receive[256];

int main()
{
	
	int fd = open("/dev/encoder_driver", O_RDWR);
	char *line;
	int ret;
	size_t len;
	
	if(fd == -1)
	{
		printf("Failed to open driver\n");
		return 1;
	}
	
	
	ret = read(fd, receive, 256);
	
	printf("Read message: %s\n", receive);
	
	close(fd);
	
	return 0;
}
