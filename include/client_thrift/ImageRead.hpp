#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/sem.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pthread.h>

using namespace cv;

typedef struct {
	char flag;
	int  size;
	char data;
}ShareData_t;


class ImageRead
{
public:
	ShareData_t *m_p;
    void * shm_addr;
    pthread_mutex_t mutex;

	ImageRead() 
	{
		m_p = NULL;
        shm_addr = NULL;
	}

	void init()
	{
		int shmid;
		std::cout << "imageRead init ok ... " << std::endl;

		// sem_id = semget((key_t)12345,1,0666|IPC_CREAT);
		// if (!set_semvalue())
		// {
		// 	fprintf(stderr, "Failed to init sem\n");
		// 	exit(EXIT_FAILURE);
		// }

		shmid = shmget((key_t)1234,640*480*2,0666|IPC_CREAT);
		if (shmid == -1)
		{
			fprintf(stderr, "shmget failed\n");
			exit(EXIT_FAILURE);
		}

		shm_addr = shmat(shmid, 0, 0);
		if (shm_addr == (void*)-1)
		{
			fprintf(stderr, "shmat failed\n");
			exit(EXIT_FAILURE);
		}

		//printf("\nMemory attached at %X\n", (int)shm_addr);
	
		m_p = (ShareData_t*)shm_addr;

		if(pthread_mutex_init(&mutex,NULL) != 0 )  
	    {  
	        printf("Init metux error.");  
	        exit(1);  
	    }
	}


	void destroy()
	{
		if (shmdt(shm_addr) == -1)
		{
			fprintf(stderr, "shmdt failed\n");
			exit(EXIT_FAILURE);
		}

		pthread_mutex_destroy(&mutex);
	}

	void read_data(Mat *rgbImg)
	{
		int val = pthread_mutex_lock(&mutex);
		if(val != 0)  
		{  
	        printf("lock error.");  
	    }  
		while (true) {

			// printf("try get data %d\n",m_p->flag);
//			std::cout << "mp flag: " << m_p->flag << std::endl;
			if (m_p->flag != 0)
			{
				convert(rgbImg);
				m_p->flag = 0;
				break;
			}

			usleep(30000);
		}
		pthread_mutex_unlock(&mutex);

	}


	void convert(Mat *rgbImg)
	{
		int width = 640;
		int height = 480;

		unsigned char * pYuvBuf = (unsigned char *)&(m_p->data);

		cv::Mat yuvImg(height*1.5,width,CV_8UC1,pYuvBuf);
						
		cv::cvtColor(yuvImg,*rgbImg,CV_YUV2BGR_I420);

	}
};

