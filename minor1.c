#include <limits.h>//for INT_MAX
#include <stdio.h>
#include <stdbool.h>//for using boolean data type
#include <time.h>
//#include <windows.h>
//#include <dir.h>
//#include <dos.h>
#include <stdlib.h>
#define MAX 100

#define QUEUE_SIZE 50
#define INFINITY 9999

int lower=1;
int upper=9;
int graph[MAX][MAX];
int primgraph[MAX][MAX] = { 0 };
int n,starting_vertex,end_vertex;
clock_t start1, end1, start2, end2, start3, end3;
double cpu_time_used_bfs, cpu_time_used_dijkstra,cpu_time_used_astar;
static int count_bfs=0,total_weight_bfs=0,count_dij=1,total_weight_dij=0,count_astar=1, total_weight_astar=0;
//int endDistance[MAX];
//n is number of nodes
//starting_vertex is starting node
//end_vertex is ending node

//queue
int queue[QUEUE_SIZE];
int queue_front, queue_end;
void enqueue(int v);
int dequeue();

//function declarations
void adjMatrixOfGraph();


/* ------------------------------CONSTRUCTION OF MAZE ( 2-DIMENSIONAL MATRIX )------------------------------------------*/

void adjMatrixOfGraph()
{
    int i,j;
	top:
    printf("\nEnter the number of vertices: ");
    scanf("%d",&n);
	if(n>50)
	{
		printf("Please enter number of vertices between 0 and 50");
		goto top;
	}
    printf("\n");
    //n = n*n;
    //SetColor(3);
    for(i=0;i<n;i++)
    {
        for(j=0;j<n;j++)
        {
            if(i==j)
            {
                graph[i][j]=0;
                continue;
            }
            graph[i][j]=(rand() %(upper - lower + 1)) + lower;
            graph[j][i] = graph[i][j];

        }
    }
    printf(" ");
    for(i=0;i<n;i++)
    {
        printf("    %c",65+i);
    }
    printf("\n");
    for(i=0;i<n;i++)
    {
        printf("%c",65+i);
        for(j=0;j<n;j++)
        {
            printf("    %d",graph[i][j]);
        }
            printf("\n");
    }
}
int print(int parent[])
{
    //SetColor(15);
  printf("\nparent\tEdge \tWeight\n");
  for (int i = 0; i < n; i++)
  {
    printf(" %c  -  %c  \t%d \n", parent[i]+65, i+65, graph[i][parent[i]]);
  }


    printf("\n\nFinal Maze:\n\n");
    //SetColor(5);
  printf(" ");
    for(int i=0;i<n;i++)
    {
        printf("    %c",65+i);
    }
    printf("\n");
    for(int i=0;i<n;i++)
    {
        printf("%c",65+i);
        for(int j=0;j<n;j++)
        {
            printf("    %d",primgraph[i][j]);
        }
            printf("\n");
    }

}

/*---------------------------------PATH GENERATION via RANDOMIZED PRIMS ALGORITHM------------------------------------------*/

void randPrims()
{
  int values[n];
  int parent[n];
  bool visited[n];

  //initializing values and visited arrays
  for(int i = 0; i<n; i++)
  {
    visited[i]= false;
    values[i]=INT_MAX;
  }
  //initializing first node

  //parent of first node is nil
  parent[0] = -1;
  //initializing value to first element as 0
  values[0] = 0;
  //first element is always visited
  visited[0] = true;

  srand(time(0));
  for(int i=0; i<n-1; i++)
  {
  //to generate random values at every iteration
    //randomly selected current node
    int y = rand()%n;
    //randomly selected child node after y
    int u = rand()%n;

    //check whether selected random node is in visited array or not.
    while(visited[y]!= true)
    {y = rand()%n;}

    //check whether selected random node is in non visited array or not.
    while(visited[u]==true)
    {u = rand()%n;}

    //make randomly chosen node as visited.
    visited[u] = true;
    //assign parent of u as y
    parent[u] = y;
    //assign value to the link between u and y
    values[u] = graph[u][y];
    primgraph[u][y] = graph[u][y];
    primgraph[y][u]=graph[u][y];
    //printf("%d...\n",primgraph[u][y]);
  }

  print(parent);
}

/*--------------------------BREADTH FIRST SEARCH (BFS) ALGORITHM IMPLEMENTATION----------------------------------------*/

void bfs(int Adj[][MAX], int n, int source, int dest) {
  //variables
  int i, j;
    int parent[n];
    //visited array to flag the vertex that
  //were visited
  int visited[MAX];

  //queue
  queue_front = 0;
  queue_end = 0;

  //set visited for all vertex to 0 (means unvisited)
  for(i = 0; i < MAX; i++) {
    visited[i] = 0;
  }

  //mark the visited source
  visited[source] = 1;

  //enqueue visited vertex
  enqueue(source);

  //print the vertex as result
  //printf(" --> %c ", source+65);

  //continue till queue is not empty
  while(queue_front <= queue_end) {
    //dequeue first element from the queue
    i = dequeue();

    for(j = 0; j < n; j++) {
      if(visited[j] == 0 && Adj[i][j]!=0) {
        //mark vertex as visitedprimgraph
        parent[j] = i;
        visited[j] = 1;
        count_bfs++;
       //total_weight_bfs = total_weight_bfs + primgraph[i][j];

        //push vertex into queue
        enqueue(j);


        //print the vertex as result
        //printf("--> %c ", j+65);
//        if(j == dest)
//        {
//            goto next;
//        }

      }
    }
  }

  int temp,distance=0;
  int k = dest;
  while(k != source)
  {
      temp = parent[k];
      distance = distance + primgraph[temp][k];
      printf("%c <-- ",k+65);
      k = parent[k];

  }
  total_weight_bfs = distance;

    printf("%c",source + 65);
  next:
  printf("\n");
}

void enqueue(int v) {
  //deals with insertion in the queue
  queue[queue_end] = v;
  queue_end++;
}

int dequeue() {
    //deals with the deletion of elements in a queue
  int index = queue_front;
  queue_front++;
  return queue[index];
}

/*--------------------------------DIJKSTRA'S ALGORITHM IMPLEMENTATION------------------------------------------*/

void dijkstra(int G[MAX][MAX],int n,int startnode, int endnode)
{

	int cost[MAX][MAX],distance[MAX],pred[MAX];
	int visited[MAX],count,mindistance,nextnode,i,j;

	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//cost[][] create the cost matrix

	for(i=0;i<n;i++)
  {
		for(j=0;j<n;j++)
    {
			if(G[i][j]==0)
      {
				cost[i][j]=INFINITY;
      }
			else
      {
				cost[i][j]=G[i][j];
      }
    }
  }
	//initialize pred[],distance[] and visited[]
	for(i=0;i<n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}

	distance[startnode]=0;
	visited[startnode]=1;
	count=1;


  start2=clock();
  //start2=start2*1000;

  while(count<n-1)
	{
		mindistance=INFINITY;

		//nextnode gives the node at minimum distance, finding minimum distance neighbour
		for(i=0;i<n;i++)
    		{
			if(distance[i] < mindistance && !visited[i])
			{
				mindistance = distance[i];
				nextnode = i;
				//count_dij++;
			}
    		}
			//check if a better path exists through nextnode
			visited[nextnode] = 1;



      for(i=0;i<n;i++)
      {
				if(!visited[i])
        			{
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
						count_dij++;
						
					}
        			}
      }
      				total_weight_dij=distance[endnode];
		  count++;
	}

	//print the path and distance of each node
	for(i=0;i<n;i++)
  {
		if(i==endnode)
		{
			
			printf("\nDistance of node %c from %c = %d",i+65,startnode+65,distance[i]);
			printf("\n\nPath:\n %c",i+65);

			j=i;
			do
			{
				j=pred[j];
				printf(" <-- %c ",j+65);

			}while(j!=startnode);
    }
  }
  end2=clock();
        //end2=end2*1000;
         cpu_time_used_dijkstra = ((double) (end2 - start2)) / CLOCKS_PER_SEC;

         cpu_time_used_dijkstra = cpu_time_used_dijkstra * 1000;
}

/*--------------------------------AStar'S ALGORITHM IMPLEMENTATION------------------------------------------*/

int* finalDistance(int G[MAX][MAX],int n,int startnode, int endnode)
{
  static int distance[MAX];
	int cost[MAX][MAX],pred[MAX];
	int visited[MAX],count,mindistance,nextnode,i,j;

	//pred[] stores the predecessor of each node
	//count gives the number of nodes seen so far
	//cost[][] create the cost matrix

	for(i=0;i<n;i++)
  {
		for(j=0;j<n;j++)
    {
			if(G[i][j]==0)
      {
				cost[i][j]=INFINITY;
      }
			else
      {
				cost[i][j]=G[i][j];
      }
    }
  }
	//initialize pred[],distance[] and visited[]
	for(i=0;i<n;i++)
	{
		distance[i]=cost[startnode][i];
		pred[i]=startnode;
		visited[i]=0;
	}

	distance[startnode]=0;
	visited[startnode]=1;
	count=1;

	while(count<n-1)
	{
		mindistance=INFINITY;

		//nextnode gives the node at minimum distance
		for(i=0;i<n;i++)
    {
			if(distance[i] < mindistance && !visited[i])
			{
				mindistance = distance[i];
				nextnode = i;
			}
    }
			//check if a better path exists through nextnode
			visited[nextnode] = 1;
			for(i=0;i<n;i++)
      {
				if(!visited[i])
        {
					if(mindistance+cost[nextnode][i]<distance[i])
					{
						distance[i]=mindistance+cost[nextnode][i];
						pred[i]=nextnode;
					}
        }
      }
		  count++;
	}
	//returns the path and distance of each node
	return distance;

}

void AStar(int G[MAX][MAX],int n,int startnode, int endnode)
{
  int flag = 1;
  int* heuristic;
  struct node
  {
    int globalGoal;
    int localGoal;
    int parent;
    bool visited;
  };

  struct node v[n];

  for(int i = 0; i < n; i++)
  {
    v[i].globalGoal = INFINITY;
    v[i].localGoal = INFINITY;
    v[i].parent = -1;
    v[i].visited = false;
  }

  heuristic = finalDistance(primgraph,n,end_vertex,starting_vertex);
/*
  for(int i=0;i<n;i++)
  {
    int u=heuristic[i]+2;
    int l=heuristic[i]-2;
   // heuristic[i]=(rand() %(u - l + 1)) + l;
  }
*/  
printf("\nHeuristic Values: \n");
  for(int i = 0; i<n; i++)
  {
    printf("%d    ", *(heuristic+i));
  }
  //printf("%d",heuristic[0]);

  //initializing starting conditions
start3=clock();
//start3=start3*1000.00f;


  v[startnode].localGoal = 0;
  v[startnode].globalGoal = heuristic[startnode];
  v[startnode].visited = true;

  int currentnode = startnode;

  //checking if the list is empty or not

  //proceeding with the algorithm for other nodes as well
    do
    {
      for(int i=0; i<n; i++)
      {
        if(!v[i].visited)
        {
          flag = 0;//all nodes are not visited.
          break;
        }
      }

      for(int i=0; i<n; i++)
      {
        if(G[currentnode][i]!=0)
        {
          if(v[i].localGoal > v[currentnode].localGoal + G[currentnode][i])
          {
            v[i].localGoal = v[currentnode].localGoal + G[currentnode][i];
            v[i].globalGoal = heuristic[i] + v[i].localGoal;
            count_astar++;
          }
        }
      }

      int mindistance=INFINITY;
      int nextnode=-1;
  		for(int i=0;i<n;i++)
      {
        if(G[currentnode][i]!=0)
        {
          if(v[i].globalGoal < mindistance && !v[i].visited)
          {
            mindistance = v[i].globalGoal;
            nextnode = i;
          }
        }
        if(nextnode==endnode)
        {
          continue;
        }
      }
      v[nextnode].visited = true;
      v[nextnode].parent = currentnode;
      currentnode = nextnode;
    }while(flag == 0 && currentnode!=endnode);
    int i = endnode;
    printf("\n\nA-star Path:\n");
    if(i!=startnode)
    {
        printf(" %c",i+65);
    }
    while(i!=startnode)
    {
        total_weight_astar= total_weight_astar+ G[i][v[i].parent];
      printf(" <-- %c",(v[i].parent)+65);
      i = v[i].parent;

    }

    end3 = clock();
    //end3=end3*1000.00f;
        cpu_time_used_astar = ((double) (end3 - start3)) / CLOCKS_PER_SEC;

        cpu_time_used_astar = cpu_time_used_astar * 1000;
}

/*-------------------------------Comparison------------------------------------------*/

void compare(){
printf("\n\n**************************COMPARISON OF ALGORITHMS**************************");
printf("\n\nBFS\n   Time Taken: %lf \n   Blocks Computed: %d \n   Total Distance: %d",cpu_time_used_bfs,count_bfs,total_weight_bfs);
printf("\n\nDijkstra\n   Time Taken: %lf\n   Blocks Computed: %d \n   Total Distance: %d ",cpu_time_used_dijkstra,count_dij,total_weight_dij);
printf("\n\nA*\n   Time Taken: %lf \n   Blocks Computed: %d \n   Total Distance: %d ",cpu_time_used_astar, count_astar,total_weight_astar);
printf("\n\n");


}


int main()
{
    //SetColor(14);
    printf("======================================================================================================================");
    printf("\n\t\t\t-------- LABYRINTH : A COMPARATIVE ANALYSIS OF PATH FINDING ALGORITHMS --------\t\t\n");
    printf("======================================================================================================================\n\n");

    //SetColor(15);
    //creates the initial adjacency matrix for maze generation
    adjMatrixOfGraph();
    //creation of maze and a system-defined path
    randPrims();
    //user needs to enter the starting and ending vertex
    reenter:
    //SetColor(15);

    printf("\n\nEnter the starting vertex:");
    scanf("%d",&starting_vertex);
    printf("\nEnter the ending vertex:");
    scanf("%d",&end_vertex);

    if(starting_vertex < 0 || starting_vertex > n || end_vertex < 0 || end_vertex > n)
    {
      //  SetColor(4);
        printf("\nINVALID INPUT !!     PLEASE ENTER THE VALUES AGAIN .....");
        goto reenter;
    }

    printf("\n\n");

    //SetColor(14);
    printf("************** Breadth First Search Algorithm Path **************\n\n");
    //Final output after implementation of BFS algorithm
    //SetColor(15);
    printf("BFS Path :\n\n");
    start1 = clock();
    //start1=start1*1000.00f;
    bfs(primgraph,n,starting_vertex,end_vertex);
    end1 = clock();
    //end1=end1*1000.00f;
    cpu_time_used_bfs = ((double) (end1 - start1)) / CLOCKS_PER_SEC;
    cpu_time_used_bfs = cpu_time_used_bfs * 1000;

    //SetColor(14);
    printf("\n\n\n******************* Dijkstra's Algorithm Path *******************\n\n");
   // SetColor(15);
    printf("Dijkstra's Path:\n");
    dijkstra(primgraph,n,starting_vertex,end_vertex);

    //SetColor(14);
    printf("\n\n\n******************* AStar's Algorithm Path *******************\n\n");
    //SetColor(15);
    printf("AStar's Path:\n");
    AStar(primgraph,n,starting_vertex,end_vertex);
   compare();
  return 0;
}
/*
 void SetColor(int ForgC)
 {
  WORD wColor;

   HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
   CONSOLE_SCREEN_BUFFER_INFO csbi;

                        //We use csbi for the wAttributes word.
  if(GetConsoleScreenBufferInfo(hStdOut, &csbi))
  {
                  //Mask out all but the background attribute, and add in the forgournd color
       wColor = (csbi.wAttributes & 0xF0) + (ForgC & 0x0F);
       SetConsoleTextAttribute(hStdOut, wColor);
  }
  return;
}
*/
