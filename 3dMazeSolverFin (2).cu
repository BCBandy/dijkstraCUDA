#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <sys/time.h>
//#include <time.h>

typedef struct vertex vertex;

struct vertex {
	int _3dCoordinate[3];	//position xyz
	int adjEdges[5][3];
	long long adjIndex[5];
	long index;
	long visited;
	long dist;
	long prev;
};

typedef struct node node;

struct node{
	vertex v;
	node *next;
	node *prev;
};

struct timezone Idunno;	
struct timeval startTime, endTime;

//graph
int entrance[3];
int mazeExit[3];
vertex *vertices;
char *pathToExit;
int degreeOfRandom = 3;

//dijkstra
//long long _INFINITY = 9223372036854775807, totalCount, numEdges = 0;
int _INFINITY = 2147483647, totalCount, numEdges = 0;
node *head;
node *current;

//general
int mazeX = 3, mazeY = 3, mazeZ;
long long numVertices;
int initTmp = 7;

//cuda data
int *mSum;// = &initTmp;	//iterator control for gpu dijkstra
int gpuPathLength;
int numThreads = 1024;


int *E;	//edge array
int *S;	//start position
int *D; 	//degree
int *M;	//mask
int *C;	//cost array
int *U;	//update cost array
int *gpuPath;

int *E_d;
int *S_d;
int *D_d;
int *M_d;
int *C_d;
int *U_d;
int *path_d;
int *mSum_d;

long long *numVertices_d;

void makeGraph(){
	//downCount keeps track of when to add an open floor to the maze
	long long downCount = 0;
	int x = 0, y = 0, z = 0;
	totalCount = 0;
	
	//build maze with known long path to exit with random elements
	//set up all adjacent edges for each vertex
	for(z = 0; z < mazeZ; z++){
		for(y = 0; y < mazeY; y++){
			for(x=0; x < mazeX; x++){
				int a, b;
				
				
				//initlize adjEdges to -1
				for (a = 0; a < 5; a++){
					for(b = 0; b < 3; b++){
						vertices[totalCount].adjEdges[a][b] = -1;
					}
					vertices[totalCount].adjIndex[a] = -1;
				}
				vertices[totalCount].visited = 0;

				vertices[totalCount]._3dCoordinate[0] = x;
				vertices[totalCount]._3dCoordinate[1] = y;
				vertices[totalCount]._3dCoordinate[2] = z;

				//add east adj edge to current vertex
				if(x == 0 && y == 0){
					vertices[totalCount].adjEdges[2][0] = x+1;
					vertices[totalCount].adjEdges[2][1] = y;
					vertices[totalCount].adjEdges[2][2] = z;
					/*
					if(totalCount+1 < numVertices){
						vertices[totalCount].adjIndex[2] = vertices[totalCount+1];
					}
					*/
					
					vertices[totalCount].adjIndex[2] = totalCount+1;

					numEdges++;
					downCount++;
				}
				//add east, west adj edge to current vertex
				else if(x != 0 && x != mazeX-1 && y == 0){
					//east
					vertices[totalCount].adjEdges[2][0] = x+1;
					vertices[totalCount].adjEdges[2][1] = y;
					vertices[totalCount].adjEdges[2][2] = z;
					/*
					if(totalCount+1 < numVertices){
						vertices[totalCount].adjIndex[2] = vertices[totalCount+1];
					}
					*/
					vertices[totalCount].adjIndex[2] = totalCount+1;
					
					//west
					vertices[totalCount].adjEdges[3][0] = x-1;
					vertices[totalCount].adjEdges[3][1] = y;
					vertices[totalCount].adjEdges[3][2] = z;
					/*
					if(totalCount-1 > -1){
						vertices[totalCount].adjIndex[3] = vertices[totalCount-1];
					}
					*/
					vertices[totalCount].adjIndex[3] = totalCount-1;
					numEdges+=2;
				}
				//add north, west adj edge to current vertex
				else if(x == mazeX-1 && y == 0){
					//north
					vertices[totalCount].adjEdges[0][0] = x;
					vertices[totalCount].adjEdges[0][1] = y+1;
					vertices[totalCount].adjEdges[0][2] = z;
					/*
					if (totalCount + mazeX < numVertices){
						vertices[totalCount].adjIndex[0] = vertices[totalCount+mazeX];
					}
					*/
					vertices[totalCount].adjIndex[0] = totalCount+mazeX;
					
					//west
					vertices[totalCount].adjEdges[3][0] = x-1;
					vertices[totalCount].adjEdges[3][1] = y;
					vertices[totalCount].adjEdges[3][2] = z;
					/*
					if(totalCount > -1){
						vertices[totalCount].adjIndex[3] = vertices[totalCount-1];
					}
					*/
					vertices[totalCount].adjIndex[3] = totalCount-1;
					numEdges+=2;
				}
				//add north, south adj edge to current vertex
				else if(x == mazeX-1 && y != mazeY-1 && y != 0){
					//north
					vertices[totalCount].adjEdges[0][0] = x;
					vertices[totalCount].adjEdges[0][1] = y+1;
					vertices[totalCount].adjEdges[0][2] = z;
					/*
					if(totalCount+mazeX < numVertices){
						vertices[totalCount].adjIndex[0] = vertices[totalCount+mazeX];
					}
					*/
					vertices[totalCount].adjIndex[0] = totalCount+mazeX;
					
					//south
					vertices[totalCount].adjEdges[1][0] = x;
					vertices[totalCount].adjEdges[1][1] = y-1;
					vertices[totalCount].adjEdges[1][2] = z;
					/*
					if(totalCount - mazeX > -1){
						vertices[totalCount].adjIndex[1] = vertices[totalCount-mazeX];
					}
					*/
					vertices[totalCount].adjIndex[1] = totalCount-mazeX;
					numEdges+=2;
				}
				//add south adj edge to current vertex
				else if(x == mazeX-1 && y == mazeY-1){
					//south
					vertices[totalCount].adjEdges[1][0] = x;
					vertices[totalCount].adjEdges[1][1] = y-1;
					vertices[totalCount].adjEdges[1][2] = z;
					/*
					if(totalCount-mazeX > -1){
						vertices[totalCount].adjIndex[1] = vertices[totalCount-mazeX];
					}
					*/
					vertices[totalCount].adjIndex[1] = totalCount-mazeX;

					downCount++;
					numEdges++;
				}
				
				//remove floor to every odd plane
				if(x == 0 && y == 0){
					
					if(z % 2 != 0){
						//open floor of this vertex
						vertices[totalCount].adjEdges[4][0] = x;
						vertices[totalCount].adjEdges[4][1] = y;
						vertices[totalCount].adjEdges[4][2] = z+1;
						/*
						if (totalCount + mazeX*mazeY < numVertices){
							vertices[totalCount].adjIndex[4] = vertices[totalCount + mazeX*mazeY];
						}
						*/
						vertices[totalCount].adjIndex[4] = totalCount + mazeX*mazeY;
						numEdges++;
					}
				} 
				
				//remove floor from every even plane
				if (x == mazeX-1 && y == mazeY-1){
					
					if(z % 2 == 0){
						//open floor of this vertex
						vertices[totalCount].adjEdges[4][0] = x;
						vertices[totalCount].adjEdges[4][1] = y;
						vertices[totalCount].adjEdges[4][2] = z+1;
						/*
						if (totalCount + mazeX*mazeY < numVertices){
							vertices[totalCount].adjIndex[4] = vertices[totalCount + mazeX*mazeY];
						}
						*/
						vertices[totalCount].adjIndex[4] = totalCount + mazeX*mazeY;
						numEdges++;
					}
				}
				
				//add random walls/floors------------------------------------------------------------------
				
				int tmprand;
				tmprand = rand() % 10 + 1;
				//rand north
				if(y != mazeY-1 && tmprand <= degreeOfRandom){
					vertices[totalCount].adjIndex[0] = totalCount+mazeX;
					numEdges++;
				}
				tmprand = rand() % 10 + 1;
				//rand south
				if(y != 0 && tmprand <= degreeOfRandom){
					vertices[totalCount].adjIndex[1] = totalCount-mazeX;
					numEdges++;
				}
				tmprand = rand() % 10 + 1;
				//rand east
				if(x != mazeX-1 && tmprand <= degreeOfRandom){
					vertices[totalCount].adjIndex[2] = totalCount + 1;
					numEdges++;
				}
				tmprand = rand() % 10 + 1;
				//rand west
				if(x != 0 && tmprand <= degreeOfRandom){
					vertices[totalCount].adjIndex[3] = totalCount - 1;
					numEdges++;
				}
				tmprand = rand() % 10 + 1;
				//rand floor
				if(z != mazeZ-1 && tmprand<= degreeOfRandom){
					vertices[totalCount].adjIndex[4] = totalCount + mazeX*mazeY;
					numEdges++;
				}
			
				vertices[totalCount].index = totalCount;
				totalCount++;
			}	
		}
	}
	//printAdjIndexes();
}

double report_running_time() {
	long sec_diff, usec_diff;
	gettimeofday(&endTime, &Idunno);
	sec_diff = endTime.tv_sec - startTime.tv_sec;
	usec_diff= endTime.tv_usec-startTime.tv_usec;
	if(usec_diff < 0) {
		sec_diff --;
		usec_diff += 1000000;
	}
	printf("Running time for CPU version: %ld.%06ld sec\n", sec_diff, usec_diff);
	return (double)(sec_diff*1.0 + usec_diff/1000000.0);
}

void printAdjIndexes(){
	int i, j;

	for (i = 0; i < numVertices; i++){
		for (j = 0; j < 5; j++){
			printf("%d: %lld\n", i, vertices[i].adjIndex[j]);	
		}
	}
}

void printAdjEdges(){
	int i,k;
	
	for(i=0; i < numVertices; i++){
		for (k = 0; k < 5; k++){
			printf("%d: ", i);
			//for (j = 0; j< 3; j++){
			//	printf("%d", vertices[i].adjEdges[k][j]);
			printf("%d", vertices[i].adjIndex[k]);
			//}
			printf("\n");
		}
	}
	
	printf("total: %d\n", totalCount);
}

//print adjIndexes from linked list
void printLinked(){
	int count = 0;
	node *tmp = head;
	printf("forward traversal\n");
	while(tmp != NULL){
	/*
		for(i=0; i < 5; i++){
			printf("%d: %lld \n", count, tmp->v.adjIndex[i]);
		}
		*/
		printf("%d\n", tmp->v.index);
		count++;
		tmp=tmp->next;
	}
	printf("\n");
}

void printPath(){
	int i;
	i = numVertices-1;
	
	printf("Shortest Path: %d\n", vertices[i].dist);
	while(i != NULL){
		
	
		printf("prev: %d\n", vertices[i].index);
	
		i = vertices[i].prev;
		
	}
}

node * getNewNode(vertex v){

	node * newNode = (node*)malloc(sizeof(node));
	newNode -> v = v;
	newNode -> prev = NULL;
	newNode -> next = NULL;
	return newNode;

}

void insertAtTail(vertex v){

	node * newNode = getNewNode(v);	
		
	if(head == NULL){
		head = newNode;
		current = newNode;
		return;
	}
	
	current->next = newNode;
	newNode->prev = current;
	current = newNode;
}

void popHead(){
	node * tmp = head;

	head = head->next;
	
	if(head != NULL){
		head->prev = NULL;
	}
	free(tmp);
}

void removeNode(node* rem){
	
	node* tmp;
	
	//remove element in middle of list
	if(rem->prev != NULL && rem->next != NULL ){
		//printf("\nmiddle pop: %d\n", rem->v.index);
		//printf("prev: %d	next:%d\n", rem->prev->v.index, rem->next->v.index);
		tmp = rem->prev;
		tmp->next = rem->next;
		
		tmp = rem->next;
		tmp->prev = rem->prev;
		free(rem);
	}
	//remove element at head
	if(rem->prev == NULL && rem->next != NULL){
		//printf("\nhead pop\n");
		popHead();
	}
	//remove element at tail
	if(rem->prev != NULL && rem->next == NULL){
		//pop tail
		//printf("\ntail pop\n");
		tmp = rem->prev;
		tmp->next = NULL;
		free(rem);
	}
	//printLinked();
}

node* getShortestNode(){
	
	node *tmp = head;
	node *shortestNode;
	long long shortestDist = _INFINITY;
	
	//printf("\nget shortest node\n");
	
	while(tmp != NULL){
		//printf("%d: %lld \n", count, tmp->v.adjIndex[i]);
		if(vertices[tmp->v.index].dist < shortestDist){
			shortestDist = vertices[tmp->v.index].dist;
			shortestNode = tmp;
		}
		//count++;
		tmp=tmp->next;
	}
	//printf("shortest Node is: %lld\n", shortestNode->v.index);
	//printf("dist is: %lld", shortestNode->v.dist);
	return shortestNode;
}

void cpuDijkstra(){

	head = NULL;
	current = NULL;
	
	vertices[0].dist = 0;		//source distance
	vertices[0].prev = NULL;	//undefined
	insertAtTail(vertices[0]);
	
	int i;
	long long tmpDist;
	
	//initilize undiscovered list
	for(i=1; i < numVertices; i++){
		vertices[i].dist = _INFINITY;	//pseudo infinity, size of long long
		vertices[i].prev = NULL;	
		
		//add to linked list/undiscovered list
		insertAtTail(vertices[i]);
		
	}
	//printLinked();
	
	node * shortestNode = head;
	//while linked list not empty, do dijkstra
	while (shortestNode->next != NULL){
		shortestNode = getShortestNode();
		
		vertices[shortestNode->v.index].visited = 1;
		tmpDist = vertices[shortestNode->v.index].dist + 1;
		
		
		for(i = 0; i < 5; i++){
			
			//update dist to adjNode if less than tmpDist
			if (	tmpDist < vertices[shortestNode->v.adjIndex[i]].dist && //current dist less than updated dist
				shortestNode->v.adjIndex[i] < numVertices &&		//index is not out of range
				shortestNode->v.adjIndex[i] != -1 &&			//adjIndex is valid
				vertices[shortestNode->v.adjIndex[i]].visited == 0){	//node is unvisited
				
				vertices[shortestNode->v.adjIndex[i]].dist = tmpDist;
				vertices[shortestNode->v.adjIndex[i]].prev = shortestNode->v.index;
			}
		}
		//popHead();
		removeNode(shortestNode);
	}
	//printf("\n");
}

void getEdgeArray(){
	int i, j;
	
	E = (int*) malloc(sizeof(int)*numEdges);
	S = (int*) malloc(sizeof(int)*numVertices);
	D = (int*) malloc(sizeof(int)*numVertices);
	
	for (i = 0; i < numVertices; i++){
		S[i] = -1;
		D[i] = -1;		
	}
	
	int dCount = 0, edgeCount = 0;
	int chk = 0;
	
	for (i = 0; i < numVertices; i++){
		for (j = 0; j < 5; j++){
			if(vertices[i].adjIndex[j] != -1){
			
				if(chk == 0){
					chk = 1;
					S[i] = edgeCount;
				}
				E[edgeCount] = vertices[i].adjIndex[j];
				
				edgeCount++;
				dCount++;
			}
		}
		D[i] = dCount;
		dCount = 0;
		chk = 0;
	}
}

void printEdgeArray(){
	int i = 0;
	//long long int maxN;
	//maxN = sizeof(E)/sizeof(long long int);
	
	printf("\nE array:\n\n");
	
	for(i = 0; i < numEdges; i++){
		printf("%d: %lld \n", i, E[i]);
	}
	
	printf("\nS, D arrays:\n\n");
	
	for(i = 0; i < numVertices; i++){
		printf("%d: s: %d, d: %d\n", i, S[i], D[i]);
	}
}

void copyToGpu(){
	int i;

	//init some data
	mSum = (int*)malloc(sizeof(int));
	mSum = &initTmp;
	
	M = (int*) malloc(sizeof(int)*numVertices);
	C = (int *) malloc(sizeof(int)*numVertices);
	U = (int *) malloc(sizeof(int)*numVertices);
	
	for(i = 0; i < numVertices; i++){
		M[i] = 0;
		C[i] = _INFINITY;
		U[i] = _INFINITY;
	}
	M[0] = 1;
	C[0] = 0;
	U[0] = 0;
	
	numVertices_d = (long long*) malloc(sizeof(long long));

	
	//*numVertices_d = numVertices;
	if(cudaMalloc((void**) &numVertices_d, sizeof(long long)) != cudaSuccess){
		printf("cudaMalloc failed - turkey");
	}
	if(cudaMemcpy(numVertices_d, &numVertices, sizeof(long long), cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - turkey");
		cudaFree(numVertices_d);
	}

	//make room on gpu
	if(cudaMalloc((void**) &E_d, sizeof(int)*numEdges) != cudaSuccess){
		printf("cudaMalloc failed - E_d");
	}
	if(cudaMalloc((void**) &S_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - S_d");
	}
	if(cudaMalloc((void**) &D_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - S_d");
	}
	
	if(cudaMalloc((void**) &path_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - path_d");
	}
	/*if(cudaMalloc((void**) &gpuPath, sizeof(unsigned long long int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - gpuPath");
	}*/
	if(cudaMalloc((void**) &M_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - M_d");
	}
	if(cudaMalloc((void**) &C_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - C_d");
	}
	if(cudaMalloc((void**) &U_d, sizeof(int)*numVertices) != cudaSuccess){
		printf("cudaMalloc failed - U_d");
	}
	if(cudaMalloc((void**) &mSum_d, sizeof(int)) != cudaSuccess){
		printf("cudaMalloc failed - mSum_d");
	}
		
	//copy to gpu
	if(cudaMemcpy(E_d, E, sizeof(int)*numEdges, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - E_d");
		cudaFree(E_d);
	}
	if(cudaMemcpy(S_d, S, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - S_d");
		cudaFree(S_d);
	}
	if(cudaMemcpy(D_d, D, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - D_d");
		cudaFree(D_d);
	}
	if(cudaMemcpy(M_d, M, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - M_d");
		cudaFree(M_d);
	}
	if(cudaMemcpy(C_d, C, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - C_d");
		cudaFree(C_d);
	}
	if(cudaMemcpy(U_d, U, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - U_d");
		cudaFree(U_d);
	}
	if(cudaMemcpy(path_d, gpuPath, sizeof(int)*numVertices, cudaMemcpyHostToDevice) != cudaSuccess){
		printf("cudaMemcpy failed - U_d");
		cudaFree(U_d);
	}

}



__global__ void getMsum_kernel(int *M_d, int *mSum_d, long long *numVertices){
	long long i = blockIdx.x * blockDim.x + threadIdx.x;
	
	if(i ==0){
		*mSum_d = 0;
	}
	__syncthreads();
	
	if(i < *numVertices && M_d[i] > 0){
		atomicAdd((mSum_d), 1);
	}
}

void getMsum(){

	getMsum_kernel<<<(numVertices/256+1), 256>>>(M_d, mSum_d, numVertices_d);
	
		//get mSum from kernel
		if (cudaMemcpy(mSum, mSum_d, sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess){
			printf("cudaMemcpyDeviceToHost mSum_d failed");
		}
}

__global__ void updateCost_kernel(int * M_d, int *C_d, int *U_d, long long *numVertices){

	long long i = blockIdx.x * blockDim.x + threadIdx.x;

	if(i < *numVertices){
		if(C_d[i] > U_d[i]){
			C_d[i] = U_d[i];
			M_d[i] = 1;
		}
		//U_d[i] = C_d[i];
	}
}

__global__ void exploreNeighbors_kernel (int * E_d, int * S_d, int * D_d, int * M_d, int *C_d, int *U_d, int * path_d, long long *numVertices){

	long long i = blockIdx.x * blockDim.x + threadIdx.x;
	int j, neighborPos;
	
	if(i < *numVertices){
		if(M_d[i] == 1){
			M_d[i] = 0;
		
			//for all neighbors of node i
			for (j = 0; j < D_d[i]; j++){
		
				neighborPos = E_d[S_d[i]+j];
			
				//update weight of neighbor if less than current value
				if(U_d[neighborPos] > C_d[i] + 1){
					U_d[neighborPos] = C_d[i] + 1;
					path_d[neighborPos] = i;
				}
			}
		}
		
		__syncthreads();
		
		//update cost
		if(C_d[i] > U_d[i]){
		
			C_d[i] = U_d[i];
			M_d[i] = 1;
		}
	}
}

void getGpuPath(){
	if (cudaMemcpy(gpuPath, path_d, sizeof(int)*numVertices, cudaMemcpyDeviceToHost) != cudaSuccess){
			printf("cudaMemcpyDeviceToHost path_d failed");
		}
	//int i;	
	//for(i = 0; i < sizeof(gpuPath)/sizeof(gpuPath[0]){
}

void printGpuPath(){
	//printf("\n\nGpu Path: %d\n\n", sizeof(gpuPath)/sizeof(gpuPath[0]));
	int i = numVertices-1;
	gpuPathLength = 0;
	
	while(i != 0){
		//printf("%d\n", i);
		i = gpuPath[i];
		gpuPathLength++;
	}
	
	/*
	for(i = 0; i < numVertices; i++){
		if(gpuPath[i] != 0){
			printf("%d\n", gpuPath[i]);
		}
	}
	*/
	printf("Gpu Shortest Path: %d\n\n", gpuPathLength);
}

void printGpuFullPath(){
	int i = numVertices-1;
	gpuPathLength = 0;
	
	while(i != 0){
		printf("%d\n", i);
		i = gpuPath[i];
		gpuPathLength++;
	}
	
}

int main(int argc, char **argv){

	srand(time(NULL));

	mazeX = atoi(argv[1]);
	mazeY = atoi(argv[2]);
	mazeZ = atoi(argv[3]);
	degreeOfRandom = atoi(argv[4]);
	numThreads = atoi(argv[5]);
	
	numVertices = mazeX * mazeY * mazeZ;

	gpuPath = (int*) malloc(sizeof(int)*numVertices);	

	mazeExit[0] = mazeX -1;
	mazeExit[1] = mazeY -1;
	mazeExit[2] = mazeZ -1;

	vertices = (vertex *)malloc(sizeof(vertex)*numVertices);

	makeGraph();
	
	printf("\nNumver of Vertices: %lld", numVertices);
	printf("\nNumber of Edges: %lld\n\n", numEdges);
	
	/* start counting time */
	gettimeofday(&startTime, &Idunno);
	
	cpuDijkstra();
	
	/* check the total running time */ 
	report_running_time();
	
	printf("CPU Shortest Path: %d\n\n", vertices[numVertices-1].dist);
	//printPath();	//cpu
//---------------------------------------------------------------------------------------------------
	getEdgeArray();			//gets single array that holds all edges E for the graph
					//also makes start array S and degree array D
	//printEdgeArray();
	
	copyToGpu();
	
	cudaEvent_t start, stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start, 0);	
	
	//Do cuda dijkstra
	while(*mSum != 0){
		
		exploreNeighbors_kernel<<<(numVertices/numThreads+1), numThreads>>>(E_d, S_d, D_d, M_d, C_d, U_d, path_d, numVertices_d);
		//updateCost_kernel<<<(numVertices/numThreads+1), numThreads>>>(M_d, C_d, U_d, numVertices_d);
	
		if (cudaMemcpy(mSum, mSum_d, sizeof(int), cudaMemcpyDeviceToHost) != cudaSuccess){
			//cudaFree(mSum_d, numVertices);
			printf("cudaMemcpyDeviceToHost mSum_d failed");
		}
	
		//printf("mSum: %d\n", *mSum);
		getMsum();
		//printf("mSum: %d\n", *mSum);
		
		//printf("\n%d: mSum: %d\n",loopCounter++, *mSum);
		
	}
	
	getGpuPath();
	
	cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	float elapsedTime;
	cudaEventElapsedTime(&elapsedTime, start, stop);
	printf("Gpu time: %0.6f sec\n", elapsedTime/1000);
	cudaEventDestroy(start);
	cudaEventDestroy(stop);
	
	//printf("\n\nGpu Path: %d\n\n", (unsigned long long int) sizeof(gpuPath));
	printGpuPath();
	//printGpuFullPath();
	
	return 0;
}
