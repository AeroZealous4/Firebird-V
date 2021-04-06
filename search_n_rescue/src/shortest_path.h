#ifndef SHORTEST_PATH_H_
#define SHORTEST_PATH_H_

#define V 67	//Number of nodes
#define dest 66 //Goal node


int minDistance(int dist[],bool sptSet[]);
void dijkstra(int Dest);			//Put dest node there
void Adj_Update(int i, int j);	//i is current node and j is mid node which is not reachable 
char Next_Dir(int i, int j);//i is current node and j is ddesired node
int Next_Node(int curr_node);	//Takes value of current node and returns next node to which robot should move.
int Min_Dist_Node_Fr_Plot(int plot_no,int curr_node);//Returns node number to which u should go for respective plot
char Get_Dir_Plot(void);	////Returns direction in which center of plot is there
							//Use this fn after u reached to desired mid node which u 
							//got by calling fn Min_Dist_Node_Fr_Plot(,);
//Returns direction in which dir center of plot is there
char Get_Dir_Plot(void);
//Returns Plot Number whcih needs to be scanned
int Next_Plot_to_Scan(void); //Returns -1 if scan is complete
/* Impt: Call this fn after scan of desired plot is complete*/
void Plot_Scan_Compl(void);	//

//Put scan result in array
//Scan result: 'M':Major Injury,'m':Minor Injury,'N':No enjury,'0':Not yet scanned
void Scan_Res(int plot_nu,char inj);
char Inj_at_Plot(int plot_nu);

#endif /* EBOT_SANDBOX_H_ */
