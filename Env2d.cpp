/* 2D Environment Simulator - Env2d.cpp
   Author: Prof. Joao Alberto Fabro - UTFPR - Curitiba - PR - Brazil
   e-mail: fabro at utfpr.edu.br
   Course: Intelligent Systems - CSI30
   Version 1.0 - 20/03/2019
---------------HOW TO COMPILE AND EXECUTE ---------------------------------
   Compiling instructions: $ g++ Env2d.cpp -o Env2d
   In order to execute:    $ ./Env2d
---------------HOW TO SPECIFY THE ENVIRONMENT -----------------------------
   It is necessary to have a file "Env.txt" in the same folder, with the following structure:
   - First Line: a single integer, representing the number of rows(n_rows) of the environment;
   - Second Line: a single integer, representing the number of columns(n_cols) of the environment;
   - A sequence of 'n_rows' lines, each with 'n_cols' symbols from:
  - Character '.' representing free positions in the grid;
  - Character '*' representing positions in the grid with obstacles;
  - Character 'x' representing the 'goal' position of the 'robot';
  - One of the following characters, representing the robot's initial position and orientation:
      - Character '>' if the robot is facing EAST;
      - Character '^' if the robot is facing NORTH;
      - Character '<' if the robot is facing WEST;
      - Character 'v' if the robot is facing SOUTH;
-------------- HOW TO USE THE PROGRAM -------------------------------
Use the keyboard to control the movements of the 'robot':
  - key 'w' or 'W' to order the robot to "go forward";
  - key 'a' or 'A' to order the robot to "turn counterclockwise 45 degrees";
  - key 'd' or 'D' to order the robot to "turn clockwise 45 degrees";
  Leandro so trolla
*/


#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <iterator>
#include <iostream>

using namespace std;

#define ROBOT       26
#define WALL        27
#define TARGET      28
#define NONE        -1

#define MAX_ROWS    20
#define MAX_COLS    20

#define EAST    0
#define NORTHEAST 1
#define NORTH   2
#define NORTHWEST       3
#define WEST    4
#define SOUTHWEST       5
#define SOUTH   6
#define SOUTHEAST 7

#define CLOCKWISE    100
#define COUNTERCLOCK 101

int kbhit(void);

class Robot {
  public:
    int getX() { return locX; }
    int getY() { return locY; }
    int getFacing() { return facing; }
    void setFacing(int f) { facing = f; }
    void setLocation(int x, int y){locX=x;locY=y;};
    int move(int direction){
  if(direction==EAST)          locY++;
  else if(direction==SOUTHEAST) {locY++;locX++;}
  else if(direction==WEST)       locY--;
  else if(direction==SOUTHWEST) {locY--;locX++;}
  else if(direction==SOUTH)             locX++;
  else if(direction==NORTHWEST) {locY--;locX--;}
  else if(direction==NORTH)             locX--;
  else if(direction==NORTHEAST) {locY++;locX--;}
  };
    void print_state()
  {
    printf("Robot position: X=%d, Y=%d, facing:",locX, locY);
    if(facing==NORTH)   printf(" NORTH\n");
    if(facing==EAST)  printf(" EAST\n");
    if(facing==SOUTH)   printf(" SOUTH\n");
    if(facing==WEST)  printf(" WEST\n");
    if(facing==NORTHEAST)   printf(" NORTHEAST\n");
    if(facing==NORTHWEST)   printf(" NORTHWEST\n");
    if(facing==SOUTHEAST)   printf(" SOUTHEAST\n");
    if(facing==SOUTHWEST)   printf(" SOUTHWEST\n");
        };

  private:
    int locX;
    int locY;
    int facing;
};

class Environment {
public:
    int n_rows, n_cols;
    int Matrix[MAX_ROWS][MAX_COLS];

    void scan_state();
    void scan_state_from_file();
    void print_state();
    void move_robot(int direction);
    void rotate_robot(int clock_or_counterclock_90_degrees);
    int getValorMatrix(int i,int j){return Matrix[i][j];};
    bool getDisp(int i,int j){
      if (getValorMatrix(i,j)==-1||getValorMatrix(i,j)==28)
        return true;
      else
        return false;
    };

    bool getTarget(int i, int j){
      if (getValorMatrix(i,j)==28)
        return true;
      else
        return false;
    };
};


  Environment Env1;
  Robot Rob1;



class No{
private:
int i,j,pai_i,pai_j,position,h,valor;
float g,f;

public:
  No(int i = -1, int j = -1, int pai_i = -1, int pai_j = -1, int position = SOUTHEAST, float g = 9999, int h = 99999, float f=9999){
    this->i=i;
    this->j=j;
    this->pai_i=pai_j;
    this->pai_j=pai_j;
    this->position=position;
    this->g=g;
    this->h=h;
    this->f=f;
    valor=Env1.getValorMatrix(i,j);
  };

  int getI(){
    return i;
  }
  int getJ(){
    return j;
  }
  int getPos(){
    return position;
  }
  float getF(){
    return f;
  }

  ~No(){};

};

class Astar{
private:
  vector<No*> openList;
  vector<No*> closedList;

public:
  int calculaH(int i_inicio,int j_inicio){

    int i_dest;
    int j_dest;
    for(int x =0;x<MAX_ROWS;x++){
      for(int y =0;y<MAX_ROWS;y++){
        if(Env1.getTarget(x,y)==true){
          i_dest=x;
          j_dest=y;
          break;
        }
      }
    }
     int di = abs(i_inicio-i_dest);
     int dj = abs(j_inicio-j_dest);
     float h;
     if(di>dj){
      h=1.5*dj+1*(di-dj);
     }
     else{
       h=1.5*di+1*(dj-di);
     }
     return h;
  };




  void busca(){

    for(int x =0;x<MAX_ROWS;x++){
      for(int y =0;y<MAX_ROWS;y++){
        if(Env1.getTarget(x,y)==true){
          No *nodeFinal= new No(x,y);
          break;
        }
      }
    }

    int i_inicio=Rob1.getX();
    int j_inicio=Rob1.getY();
    int auxPosition=Rob1.getFacing();
    int certo;

    No *nodeInicio= new No(i_inicio,j_inicio,-1,-1,auxPosition);

    openList.push_back(nodeInicio);

    float auxF=9999;

    calculaValores(nodeInicio);
    bool flag = true;

    while(flag){

      for(int cont=0;cont<openList.size();cont++){
        if(Env1.getTarget(openList[cont]->getI(),openList[cont]->getJ())){
          closedList.push_back(openList[cont]);
          int x = closedList.size();
         for(int i = 0; i < x; i++){
          printf("Caminho: %d %d", closedList[i]->getI(),closedList[i]->getJ() );
         }
          return ;
        }
      }
      for(int cont=0;cont<openList.size();cont++){
        if(openList[cont]->getF()<auxF){
          certo=cont;
          auxF=openList[cont]->getF();
        }
      }

      closedList.push_back(openList[certo]);
     
      openList.clear();
      calculaValores(openList[certo]);
    }

  }

  void percorrerLista(){

    printf("Teste\n");
    for(int aux=0; aux<closedList.size();aux++){
      std::cout<<"Caminho ";
      std::cout<<closedList[aux]->getI();
      std::cout<<"  ";
      std::cout<<closedList[aux]->getJ();
    }
  }


  void calculaValores(No *pai){

    No *aux=NULL;
    int auxH,auxPos;
    float auxG,auxF;
    int position;
    int i = pai->getI();
    int j = pai->getJ();

    if(Env1.getTarget(i,j)){
      closedList.push_back(pai);
      removeOpenList(pai);
      return;
    }


    for(int k=-1;k<2;k++){
      for(int l=-1;l<2;l++){
        if(!((i + k) < 0 || (j + l) < 0 ||  (j + l) > 19 ||  (i + k) > 19)){
          if(Env1.getDisp(i+k,j+l)){
            if(i>k+i&&j>j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1,5;
              position=7;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i>k+i&&j==j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1;
              position=6;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i>k+i&&j<j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1,5;
              position=5;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i=k+i&&j>j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1;
              position=0;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i=k+i&&j<j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1;
              position=4;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i<k+i&&j>j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1,5;
              position=1;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i<k+i&&j==j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1;
              position=2;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            else if(i<k+i&&j<j+l){
              auxH=calculaH(k+i,l+j);
              auxG=1,5;
              position=3;
              auxG+=calculaRotacao(pai,position);
              auxF=auxG+auxH;
            }
            aux= new No(k+i,l+j,i,j,position,auxG,auxH,auxF);

            if(!isCloseList(aux)){
              openList.push_back(aux);
              printf("Aux %d %d\n", aux->getI(), aux->getJ());
            }
          }
        }
      }
    }
  }

  int calculaRotacao(No* node, int pos){

    int menor=9999;
    int x=node->getPos();
    int cl,co;
    if(pos<x){
      co=abs(pos-x);
      cl=abs(pos-x-8);
    }
    else{
      co=abs(pos-x+8);
      cl=abs(pos-x);
    }

    if(co<=cl)
      return co;
    else
      return cl;
  }

  bool getOpenlist(No *node)
  {
    for (int i = 0; i < openList.size(); i++)
    {
      if (node == openList[i])
      {
        return true;
      }
    }

    return false;
  }

  int getPositionOpenList(No *node)
  {
    for (int i = 0; i < openList.size(); i++)
    {
      if (node == openList[i])
      {
        return i;
      }
    }

    return -1;
  }

  void removeOpenList(No *node)
  {
    openList.erase(openList.begin() + getPositionOpenList(node) - 1);
  }

  bool isCloseList(No *node)
  {
    for (int i = 0; i < closedList.size(); i++)
    {
      if (node->getI() == closedList[i]->getI() && node->getJ() == closedList[i]->getJ())
      {
        return true;
      }
    }

    return false;
  }

};



int main( void )
{


  // scan the state of the environment
  Env1.scan_state_from_file();

  // print the state of the environment
  Env1.print_state();

  Astar maze;
  maze.busca();

  return 0;
}


void Environment::scan_state_from_file()
{
int r,c;
char url[]="Env.txt";
char ch;
FILE *arq;

arq = fopen(url, "r");
if(arq == NULL)
    printf("Error, it is not possible to open the Env.txt file!\n");
fscanf(arq, "%d\n", &n_rows);
fscanf(arq, "%d\n", &n_cols);
for(r=0;r<n_rows;r++)
  {
      for(c=0;c<n_cols;c++)
      {
        ch=fgetc(arq);
   //     printf("%d,%d = %c\t",r,c,ch);
    if ( ch == '*' ) {
      Matrix[r][c] = WALL;
          }
    else if ( ch == 'x' ) {
      Matrix[r][c] = TARGET;
          }
    else if ( ch == '^' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(NORTH);
      }
    else if ( ch == '>' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(EAST);
          }
    else if ( ch == 'v' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(SOUTH);
      }
    else if ( ch == '<' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(WEST);
      }
    else {
      Matrix [r][c] = NONE;
      }//end of if...else
    }//end of for(c)
  fscanf(arq,"\n");// to cath the new line
  }//end of for(r)
fclose(arq);
}

void Environment::scan_state()
{
  int ch;
  int r,c;


  printf("Number of Lines:");
  scanf("%d\n", &n_rows);
  printf("Number of Columns:");
  scanf("%d\n", &n_cols);
  for(r=0;r<n_rows;r++)
   {
    for(c=0;c<n_cols;c++)
    {

     ch=getchar();
//     printf("%d,%d = %c\t",r,c,ch);
     if ( ch == '*' ) {
      Matrix[r][c] = WALL;
     }
     else if ( ch == 'x' ) {
      Matrix[r][c] = TARGET;
     }
     else if ( ch == '^' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(NORTH);
     }
     else if ( ch == '>' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(EAST);
     }
     else if ( ch == 'v' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(SOUTH);
     }
     else if ( ch == '<' ) {
      Matrix [r][c] = ROBOT;
      Rob1.setLocation(r,c);
      Rob1.setFacing(WEST);
     }
     else {
      Matrix [r][c] = NONE;
     }//end of if...else
    }//end of for(c)
    scanf("\n");// to cath the new line
   }//end of for(r)
}


void Environment::print_state()
{

int r,c;
 printf("    ");
 for(int i=0;i<n_cols;i++)
   printf("  %d ",i);
 printf("\n");
 printf("    ");
 for(int i=0;i<n_cols;i++)
   printf("+---");
 printf("+\n");
 for( r=0; r < n_rows ; r++ )
 {
 printf("  %d ",r);
   for( c=0; c < n_cols; c++ ) {
      if (Matrix[r][c] == WALL)
       {
        printf( "|***" );
       }

      if (Matrix[r][c] == TARGET)
       {
        printf( "| X " );
       }

      if (Matrix[r][c] == NONE)
       {
        printf( "|   " );
       }

      if ( Matrix[r][c] == ROBOT )
       {
        if (Rob1.getFacing() == NORTH)
        {
          printf("| ^ ");
        }
        else if (Rob1.getFacing() == NORTHEAST)
        {
          printf("| / ");
        }
        else if (Rob1.getFacing() == EAST)
        {
          printf("| > ");
        }
        else if (Rob1.getFacing() == SOUTHEAST)
        {
          printf("| \\ ");
        }
        else if (Rob1.getFacing() == SOUTH)
        {
          printf("| v ");
        }
        else if (Rob1.getFacing() == SOUTHWEST)
        {
          printf("| %% ");
        }
        else if (Rob1.getFacing() == WEST)
        {
          printf("| < ");
        }
  else if (Rob1.getFacing() == NORTHWEST)
        {
          printf("| # ");
        }//end of if(Rob1...)else
      }//end of if(Matrix....)

   }//end of for(c)
   printf( "|\n" );
   printf("    ");
   for(int i=0;i<n_cols;i++)
     printf("+---");
   printf("+\n");
 }//end of for(r)
 Rob1.print_state();
}


void Environment::move_robot(int direction)
{
  if ((direction == NORTH) && (Rob1.getFacing() == NORTH) && (Matrix[Rob1.getX()-1][Rob1.getY()]!=WALL) && (Rob1.getX() > 0))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()-1][Rob1.getY()] = ROBOT;
    Rob1.move(NORTH);
  }
  else
  if ((direction == NORTHWEST) && (Rob1.getFacing() == NORTHWEST) && (Matrix[Rob1.getX()-1][Rob1.getY()-1]!=WALL) && (Rob1.getX() > 0) && (Rob1.getY() > 0))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()-1][Rob1.getY()-1] = ROBOT;
    Rob1.move(NORTHWEST);
  }
       else
  if ((direction == SOUTH) && (Rob1.getFacing() == SOUTH) && (Matrix[Rob1.getX()+1][Rob1.getY()]!=WALL) && (Rob1.getX() < n_rows-1))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()+1][Rob1.getY()] = ROBOT;
    Rob1.move(SOUTH);
  }
  else
  if ((direction == SOUTHEAST) && (Rob1.getFacing() == SOUTHEAST) && (Matrix[Rob1.getX()+1][Rob1.getY()+1]!=WALL) && (Rob1.getX() < n_rows-1) && (Rob1.getY() < n_cols-1))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()+1][Rob1.getY()+1] = ROBOT;
    Rob1.move(SOUTHEAST);
  }
  else
  if ((direction == EAST) && (Rob1.getFacing() == EAST) && (Matrix[Rob1.getX()][Rob1.getY()+1]!=WALL) && (Rob1.getY() < n_cols-1))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()][Rob1.getY()+1] = ROBOT;
    Rob1.move(EAST);
  }
  else
  if ((direction == SOUTHWEST) && (Rob1.getFacing() == SOUTHWEST) && (Matrix[Rob1.getX()+1][Rob1.getY()-1]!=WALL) && (Rob1.getX() < n_rows-1) && (Rob1.getY() >0))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()+1][Rob1.getY()-1] = ROBOT;
    Rob1.move(SOUTHWEST);
  }
        else
  if ((direction == WEST) && (Rob1.getFacing() == WEST) && (Matrix[Rob1.getX()][Rob1.getY()-1]!=WALL) && (Rob1.getY() > 0))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()][Rob1.getY()-1] = ROBOT;
    Rob1.move(WEST);
  }
  else
  if ((direction == NORTHEAST) && (Rob1.getFacing() == NORTHEAST) && (Matrix[Rob1.getX()-1][Rob1.getY()+1]!=WALL) && (Rob1.getX() >0) && (Rob1.getY() < n_cols-1))
  {
    Matrix[Rob1.getX()][Rob1.getY()] = NONE;
    Matrix[Rob1.getX()-1][Rob1.getY()+1] = ROBOT;
    Rob1.move(NORTHEAST);
  }

}

void Environment::rotate_robot(int clock_or_counterclock_90_degrees)
{
int local_facing;
   if(clock_or_counterclock_90_degrees == CLOCKWISE)
   {
  local_facing = Rob1.getFacing();
  local_facing--;
  if(local_facing<0) local_facing = 7;
  Rob1.setFacing(local_facing);
  }
   else
   if(clock_or_counterclock_90_degrees == COUNTERCLOCK)
   {
  local_facing = Rob1.getFacing();
  local_facing++;
  if(local_facing>7) local_facing = 0;
  Rob1.setFacing(local_facing);
   }
}


    int kbhit(void)
    {
      struct termios oldt, newt;
      int ch;
      int oldf;

      tcgetattr(STDIN_FILENO, &oldt);
      newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
      fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

      ch = getchar();

      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      fcntl(STDIN_FILENO, F_SETFL, oldf);

      if(ch != EOF)
      {
        ungetc(ch, stdin);
        return 1;
      }

      return 0;
    }