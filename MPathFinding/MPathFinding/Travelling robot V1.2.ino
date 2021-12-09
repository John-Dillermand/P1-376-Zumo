 #include <Wire.h>
//#include <Zumo32U4.h>



//const int numDest = 10; //amount of destinations. tested up to 18. took around 2 seconds on arduino UNO (~100 times less processing than zumo)
//With 16 spots it took around 0.15 secs if all print functionality and delay was removed.
//Does not alway come with optimal solution
struct Destination{
    char id; //character to identify the destination
    int x;//x-coordinate
    int y;//y-coordinate
};


struct Route{
    int stops[numDest+1];//Ordered array containing the tour stops. +1 so it can return to start
    double dist;
};

Route route = {{},0}; //declaration of the used route (gets edited along the way)


//Destinations with coordinates
Destination spots[numDest] ={
    {'O', 0,  0},
    {'A', 20, 40},
    {'B', 30, 70},
    {'C', 10, 80},
    {'D', 60, 20},
    {'E', 20, 30},
    {'F', 10, 20},
    {'G', 30, 40},
    {'H', 20, 20},
    {'I', 90, 10},
    {'J', 50, 60},
    {'K', 30, 70},
    {'L', 20, 100},
    {'M', 100, 100},
    {'N', 10, 100}
};

void fillDestRand(){
    char lettersNoO[25] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','P','Q','R','S','T','U','V','W','X','Y','Z'};
    spots[0].id = 'O';
    spots[0].x = 0;
    spots[0].y = 0;
    for(int i = 1; i < numDest; i++){
        spots[i].x = 10*(random() % 11);
        spots[i].y = 10*(random() % 10);
        spots[i].id = lettersNoO[i-1];
        delay(1);
    }
    Serial.println("Printing destinations");
    for(int i = 0; i < numDest; i++){
        Serial.println((String)spots[i].id + "(" + (String)spots[i].x + ", " + (String)spots[i].y + ")");
    }
}


int upperBound = 90;
//upper bound of field (used for augmented cost matrix)
double costMatrix[numDest][numDest];
//Sets up a cost matrix with direct routes between points
void setupCostMatrix(char mod)//uses d for direct and i for indirect
{   //Chooses row first
    for(int i=0; i < numDest; i++)
    {   //chooses column second
        for(int j=0; j < numDest; j++)
        {   //calculates the distance as sqrt(x^2+y^2)
            switch(mod){
                case 'd':
                    costMatrix[i][j]=sqrt(pow(spots[i].x-spots[j].x,2)+pow(spots[i].y-spots[j].y,2)); 
                    break;

                case 'i':
                    double dist1 = abs(spots[j].x-spots[i].x)+2*upperBound-(spots[i].y+spots[j].y);
                    double dist2 = abs(spots[j].x-spots[i].x)+spots[i].y+spots[j].y;
                    if (spots[i].x == spots[j].x){
                        costMatrix[i][j] = abs(spots[i].y-spots[j].y);
                        break;
                    } else if (dist1 < dist2){
                        costMatrix[i][j] = dist1;
                    } else {
                        costMatrix[i][j] = dist2;
                    }
            }
            
        }
    }
}
//Prints the cost matrix in the Serial moniter. for debugging 
void printCostMatrix(){
    for(int i=0; i < numDest; i++){//Choose row first
        Serial.print((String)spots[i].id + ": ");
        for(int j=0; j < numDest; j++){//choose column second
            if(j != numDest-1){
                Serial.print((String)costMatrix[i][j] + ", ");
            } else {
                Serial.println((String)costMatrix[i][j]);
            }
        }
    }   
}

//Uses all setup functions. Important to use the travelling robot code
void routeInit(char c){
    setupCostMatrix(c);
    //printCostMatrix();
}

//Swaps 2 integer variables' values
void swapInt(int &i, int &j){
    int temp = i;
    i=j;
    j=temp;
}

//Checks if certain index-value(from spots array) has already been visited
bool isVisited(int index){
    bool visited = false;
    for(int i = 0; i < numDest; i++){
        if(index == route.stops[i]){visited = true; break;}//breaks if index is found in route.stops
    }
    return visited;
}


//nearest neighbor solution. Used for final solution
void nnSolution(int startIndex){
        int lowestDistIndex; //placeholder variable
        int i = 0; //iterator
        int currentIndex = startIndex; //last inserted index
        route.stops[i] = startIndex; //Actually inserts index
        int remaining = numDest-1; //1 stop has already been added to the list

        while(remaining > 0){
            //An index value (not equal to one that has already been used) is chosen
            for(int i = 0; i < numDest; i++) if (isVisited(i) == false){lowestDistIndex = i; break;} 
            //for loop that compares the distance to all nonvisited stops from current stop
            for(int i = 0; i < numDest; i++){
                if(isVisited(i) == false && costMatrix[currentIndex][i] < costMatrix[currentIndex][lowestDistIndex])
                    lowestDistIndex = i;
            }
            //adds the dist to of just visited stop to total distance
            route.dist += costMatrix[currentIndex][lowestDistIndex];
            //updates last visited index
            currentIndex = lowestDistIndex;
            i++;
            remaining--;
            //the index value is added to list
            route.stops[i] = currentIndex;
            //information text (for debugging - commented out now)
            //Serial.println("Added stop " + (String)spots[currentIndex].id + ", for new total distance " + (String)route.dist);

        }
            //when all stops have been visited, travels to startpoint again
            route.dist += costMatrix[currentIndex][startIndex];
            route.stops[numDest+1] = currentIndex;
}

//Actual combined solution
void TSP(){
    //First generates path with nn-solution
    nnSolution(0);
    //prints tour for debugging
    printDest();

    bool change = true; //bool to control if values have been swapped
    
    //Repeats if a change has happened
    while(change){
        //bool is flipped to avoid infinite loop
        change = false;
        //checks for all edges if total distance is lower if end points of edges are switched
        for (int i = 0; i < numDest-2; i++){
            for (int j = i+2; j < numDest; j++){
                double oDist = costMatrix[route.stops[i]][route.stops[i+1]] + costMatrix[route.stops[j]][route.stops[j+1]];
                double nDist = costMatrix[route.stops[i]][route.stops[j]] + costMatrix[route.stops[i+1]][route.stops[j+1]];
                if(nDist < oDist){//if total distance is lower (expression is equivelant )

                    //Debug text print below
                    //Serial.println("swapping for change of: " + (String)(nDist - oDist));

                    //Subtracts difference (nDist<oDist always)
                    route.dist += nDist - oDist;
                    //switches edges (and therefore inverts the order between start and end points of edges)
                    invertPartialTour(i+1,j);
                    //change has happened so flips bool
                    change = true;
                    //A break might be necessary but no test has revealed that
                }}}
        //printDest(); //for debug purposes
    }
    //print so that we know it is done
    printDest();
    Serial.println("Route complete");
    
}

void invertPartialTour(int from, int to){
    //swaps elements from opposite ends of the partial list until they pass eachother (or hit the same value)
  while(from <= to){
    swapInt(route.stops[from], route.stops[to]);
    //Both index values go closer to middle of partial list
    from++;
    to--;
  }
}


//Self explanatory
void printDest(){
  Serial.print("Route is: ");
  for (int i = 0; i < numDest+1; i++) Serial.print(spots[route.stops[i]].id);
  Serial.println(" with distance : " + (String)route.dist);
}



void routeCal(char mode){
    routeInit(mode);
    route = {{},0};
    TSP();
    for(int i = 0; i<numDest;i++){
      wildOats[0][i] = spots[route.stops[i]].x;
      wildOats[1][i] = spots[route.stops[i]].y;
    }
}
