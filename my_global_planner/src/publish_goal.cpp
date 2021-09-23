#include <math.h>
#include <sstream>
#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_converter/costmap_converter_interface.h>

enum allapot {                                //Program szakaszai
      START           = 0,
      TERKEP_PUB      = 1,
      POLI_FOGAD      = 2,
      TERKEP_FELDOLG  = 3,
      CMAP_ELEK       = 4,
      GRID_ELKERESES  = 5,
      TRAPEZ_CELLAK   = 6,
      BOUSTROPHEDON   = 7,
      INIT_POZ_FOGAD  = 8,
      STARTCELLA      = 9,
      GRAFKERESO      =10,
      UTKERESO        =11,
      STARTPONTBA     =12,
      UTSZAMOLAS      =13,
      ELINDULAS       =14,
      BEJARAS         =15,
      TORLES          =16,
      URES            =17,
};

enum cellapar {                               //Grid térképből élek előállításához szükséges változók
      SZ_SZ   = 0,                            //Szabad-szabad
      SZ_F    = 1,                            //Szabad-foglalt
      F_SZ    = 2,                            //Foglalt-szabad
      F_F     = 3,                            //Foglalt-foglalt
};

enum esemenyek {                              //Térképezés során talált csúcsok
      NYIT  = 1,                               
      FOLY  = 2,
      ZAR   = 3,
      FOLY2 = 4,
      DEF   = 0,
};

enum irany {                                  //Bejárandó terület az éltől merre helyezkedik el
      XPLUS  = 1,                             //Függőleges él, terület jobbra
      YMINUS = 2,                             //Nem függőleges él, terület alatta
      XMINUS = 3,                             //Függőleges él, terület balra
      YPLUS  = 4,                             //Nem függőleges él, terület felette
    };

struct el {                                   //Élek csúcsait, irányát, meredekségét tároló struktúra
    geometry_msgs::Point32 csucs1;            //Él bal oldali (függőleges-> alsó) csúcsa
    geometry_msgs::Point32 csucs2;            //Él jobb oldali (függőleges-> felső) csúcsa
    irany dir;                                //Irányultság változó
    double mered;                             //Meredekség
    double a;                                 //Offszet (metszéspont számoláshoz)
}; 

struct t_cella {                              //Trapéz cella adatait tároló struktúra
    geometry_msgs::Point32  cs_bal;           //Bal alsó csúcs
    geometry_msgs::Point32 cs_jobb;           //JObb alsó csúcs
    double m_bal;                             //Bal "alap" hossza
    double m_jobb;                            //Jobb "alap" hossza
};

struct poli {                                 //Poligon cellák adatait tároló struktúra
    int db;                                   //Csúcsok száma
    geometry_msgs::Point32* cs;               //Csúcsokra mutató vektor -> memo foglalás
};

struct csucstomb{                             //Buoustrophedon cellák számolásához szükséges sturktúra
    geometry_msgs::Point32* temp;
    geometry_msgs::Point32* akt;
    int size; 
};

struct elsorszamtomb {                        //Nyitott élek halmaza: map függvényhez szükséges
    int* temp;
    int* akt;
    int  size = 0;
};

struct cellakezdet {                          //Cellák kezdeti x koordintái: map függvényhez kellenek
    double* temp;
    double* uj;
    double* regi;
    int uj_size = 0;
    int regi_size = 0;
};

struct utmatrix {                             //Be- és kilépési pontokat tároló struktúra
    int sorszam;
    bool elert;
    geometry_msgs::Point32 be;
    geometry_msgs::Point32 ki;
};

class Robot {                                 //ROBOT OSZTÁLY
  public:
    int celbaert       =0;                    //Adott poz-ba megérkezett-e már a robot
    int poli_db        =0;                    //Ennyi poligon lett megalkotva
    double atmero   =0.25;                    //Robot legnagyobb hosszúsága (m)
    int skala;                                //Robot mérete és térkép felbontása közti arány

    allapot proc_statusz = START;             //Feldolgozás lépései
    geometry_msgs::Point startpozicio;        //Robot kezdőpozíciója a bejárás előtt
    nav_msgs::OccupancyGrid terkep;           //Publikálandó, elmentett map objektum
    nav_msgs::OccupancyGrid terkep_uj;        //Csökkentett méretű térkép
    costmap_converter::ObstacleArrayMsg akad; //Akadályok struktúra
    std::vector<geometry_msgs::Pose> ut;      //Útvonal csúcspontjait tároló vektor
    el *grid_elek;                            //Térkép éleire mutató pointer
    t_cella * trapezok;                       //Trapéz cellákra mutató pointer
    t_cella *bszelek;                         //Boustrophedon cella szélei
    poli * bcellak_nagy;                      //Boustrophedon cellákra mutató pointer: méretcsökkentés előtt!!
    poli * bcellak_k;                         //Boustrophedon cellák a méretcsökkentés után 
    utmatrix * utmx;                          //Útmátrix: be- és kilépési pontok adatai

    void dilatacio();
    void poz_eltolas();
    void addNode(const geometry_msgs::Point32 poz);         
    void athaladas(int cellaidx, int utmx_idx);
    void cellatbejar(int cellaidx, int utmx_idx);
    void goalReached_Callback(const move_base_msgs::MoveBaseActionResult eredmeny);
    void initPoseReached_Callback(const geometry_msgs::PoseWithCovarianceStamped startpoz);
    void mapPublished_Callback(const nav_msgs::OccupancyGrid terkep_be);
    void polyPublished_Callback(const costmap_converter::ObstacleArrayMsg akadaly_be);
};

void Robot::dilatacio()                       //Bitmap térkép dilatálása, h ne akadjon el a robot
{
  int8_t dilatalt[(terkep.info.height*terkep.info.width)] = {}; //Hely az adatoknak
  int h = terkep.info.height;
  int w = terkep.info.width;

  for (int i = 0; i < h; i++)                           //Végig az "Oszlopokon"
  {

    for (int j = 0; j < w; j++)                         //Végig a "sorokon"
    {
      int van = 0;                                      //Van-e akadály pixel...

      if (terkep.data[i*w+j]>0) van++;                  //...az adott pixelen
      if (j < w-1 && terkep.data[i*w+j+1]>0)    van++;  //...az adott pixeltől jobbra
      if (j > 0   && terkep.data[i*w+j-1]>0)    van++;  //...az adott pixeltől balra
      if (i < h-1 && terkep.data[(i+1)*w+j]>0)  van++;  //...az adott pixel felett
      if (i > 0   && terkep.data[(i-1)*w+j]>0)  van++;  //...az adott pixel alatt

      if (van > 0) dilatalt[i*w+j] = char(100);         //Új pixel akadály
      else dilatalt[i*w+j] = char(0);                   //Új pixel szabad
    }
  }
  std::vector<signed char> d_vektor(dilatalt, dilatalt+sizeof(dilatalt));
  terkep.data = d_vektor;
}

void Robot::poz_eltolas()                     //Útvonal csúcspontjainak "beljebb"tolása
{
  for(int i = 0; i < ut.size()-1; i++)
  {

    if (ut[i].position.x == ut[i+1].position.x)               //függőleges mozgás
    {  
      if (ut[i].position.y < ut[i+1].position.y)              //+y irányú mozgás
          ut[i].position.y = ut[i].position.y + atmero/5;
      else                                                    //-y irányú mozgás
        ut[i].position.y = ut[i].position.y - atmero/5;
      
      if (ut[i].position.x > ut[i-1].position.x + atmero/5)   //előző +x irányú
        ut[i].position.x = ut[i].position.x - atmero/5;
      if (ut[i].position.x < ut[i-1].position.x -atmero/5)    //előző -x irányú
        ut[i].position.x = ut[i].position.x + atmero/5;

    }
    else if (ut[i].position.y == ut[i+1].position.y)          //vízszintes mozgás
    {
      if (ut[i].position.x < ut[i+1].position.x)              //+x irányú mozgás
        ut[i].position.x = ut[i].position.x + atmero/5;      
      else                                                    //-x irányú mozgás
        ut[i].position.x = ut[i].position.x - atmero/5;   

      if (ut[i].position.y > ut[i-1].position.y + atmero/5)   //előző +y irányú
        ut[i].position.y = ut[i].position.y - atmero/5;
      if (ut[i].position.y < ut[i-1].position.y - atmero/5)   //előző -y irányú
        ut[i].position.y = ut[i].position.y + atmero/5;
    }    
  }
}

void Robot::addNode(geometry_msgs::Point32 poz) //Új csúcs felvétele az útvonalba
{
  if (poz.x == ut.back().position.x && poz.y == ut.back().position.y)
  return;

  //Ha egy egyenesre és távolabb esik a következő pont is, akkor az előzőt ki lehet törölni.
  if ( ut.size()>1 &&
      (poz.x == ut.back().position.x && ut.back().position.x == ut[ut.size()-2].position.x && 
      ((poz.y > ut.back().position.y && ut.back().position.y > ut[ut.size()-2].position.y) ||
      (poz.y < ut.back().position.y && ut.back().position.y < ut[ut.size()-2].position.y))) ||

      (poz.y == ut.back().position.y && ut.back().position.y == ut[ut.size()-2].position.y && 
      ((poz.x > ut.back().position.x && ut.back().position.x > ut[ut.size()-2].position.x) ||
      (poz.x < ut.back().position.x && ut.back().position.x < ut[ut.size()-2].position.x))))
      ut.pop_back();

  if (ut.size()>3 && poz.x == ut[ut.size()-2].position.x && poz.y == ut[ut.size()-2].position.y &&
      ut.back().position.x == ut[ut.size()-3].position.x && ut.back().position.y == ut[ut.size()-3].position.y)
    {
      ut.pop_back();
 
    }

  ut.push_back(geometry_msgs::Pose());          //Új elem felvétele
  ut[ut.size()-1].position.x= poz.x;            //Pozíció megadása
  ut[ut.size()-1].position.y= poz.y;
  tf::Quaternion ori;                           //Orientáció megadása
  ori.setEuler(0.0,0.0,0.0);

  ut[ut.size()-1].orientation.w = ori.w();
  ut[ut.size()-1].orientation.x = ori.x();
  ut[ut.size()-1].orientation.y = ori.y();
  ut[ut.size()-1].orientation.z = ori.z();


  if (ut.size() > 1)                            //Orientáció kiszámolása: csak +-x és +- irány jön szóba most!!!!
  {
    if (ut[ut.size()-2].position.x == ut.back().position.x)   //vízszintes mozgás
    {  
      if (ut.back().position.y > ut[ut.size()-2].position.y)  //+y irányú mozgás
        ori.setEuler(0.0,0.0,1.5708);
      else                                                    //-y irányú mozgás
        ori.setEuler(0.0,0.0,-1.5708);
    }
    else if (ut[ut.size()-2].position.y == ut.back().position.y) //függőleges mozgás
    {
      if (ut.back().position.x > ut[ut.size()-2].position.x)  //+x irányú mozgás
          ori.setEuler(0.0,0.0,0.0);        
      else
        ori.setEuler(0.0,0.0,3.1415);                      //-x irányú mozgás
    }   

    ut[ut.size()-2].orientation.w = ori.w();
    ut[ut.size()-2].orientation.x = ori.x();
    ut[ut.size()-2].orientation.y = ori.y();
    ut[ut.size()-2].orientation.z = ori.z();
  }

}

class Graf {                                  //GRÁF OSZTÁLY
  
    int v;                                    //Csúcsok száma
    int e;                                    //Élek száma
    int** mx;                                 //Szomszédossági mx
  
public:

    Graf(int v, int e);                       //Szomszédossági mx létrehozás
    ~Graf();                                  //Destruktor
    void uj_el(int start, int e);             //Új él hozzáadás
    int DFS(int start, std::vector<bool>& elert, int cellak[], int db); //DFS bejárás
    int ShPath(int start,int cel, int cellak[]);  //Legrövidebb út nem szomszédos cellák közt
    bool szomszedos(int i, int j);            //Szomszédos-e két cella
};

Graf::Graf(int v, int e)                      //Üres gráf létrehozása
{
    this->v = v;
    this->e = e;
    mx = new int*[v];
    for (int sor = 0; sor < v; sor++) {
        mx[sor] = new int[v];
        for (int oszlop = 0; oszlop < v; oszlop++) {
            mx[sor][oszlop] = 0;
        }
    }
}

Graf::~Graf()                                 //Destruktor a memo felszabadításhoz
{
  for (int i=0; i<v; i++) 
  {
    delete[] mx[i];
  }
  delete[] mx;
  mx = nullptr;
}

void Graf::uj_el (int el1, int el2)           //Két csúcs közé él felvétele
{
    mx[el1][el2] = 1;
    mx[el2][el1] = 1;
}

bool Graf::szomszedos(int i, int j)           //Cellák szomszédosságának vizsgálata
{
  return this->mx[i][j];
}

int Graf::ShPath(int start,int cel,int cellak[])//Legrövidebb út függvény
{
  int ix = 0;

  int tav[this->v];       //Kimeneti tömb: minden eleme az adott csúcshoz tartozó min távolság
  bool elert[this->v];    //Igaz, ha az adott csúcs már elérve
  int elozo[this->v];     //Előző csúcsa, amiből az adott indexűt elértük
  
  for (int i = 0; i < this->v; i++)             //Inicializálás
  {
      tav[i] = this->v;
      elert[i] = false;
      elozo[i] = -1;
  }
  
  tav[start] = 0;                               //Kezdőcsúcstól vett távolság nulla.s

  for (int j = 0; j < (this->v)-1 ; j++)        //Legrövidebb utak keresése
  {
    // Pick the minimum distance vertex from the set of vertices not
    // yet processed. u is always equal to src in the first iteration.   
  
    int min = this->v;
    int min_index;

    for (int k = 0; k < (this->v); k++)         //végig minden csúcson:
      if (elert[k] == false && tav[k] <= min)   //Ha még nincs elérve a csúcs és minimális a táv
          min = tav[k], min_index = k;          //Csúcs vizsgálata.
    
    elert[min_index] = true;                    //A min távra lévő csúcs elérésének jelzése.

    for (int k = 0; k < this->v; k++)           //Többi csúcs távolságának frisítése
    {
      //Csúcs frissítése, ha még nem elért, van köztük él és a táv+ él összege 
      //kisebb, mint az eddigi összeg

      if (elert[k]==0 && this->mx[min_index][k] && tav[min_index] != this->v
          && tav[min_index] + this->mx[min_index][k] < tav[k])
        {
          tav[k] = tav[min_index] + this->mx[min_index][k];
          elozo[k] = min_index;
        }
    }
  }
  ix = tav[cel];
  cellak[ix-1] = cel;
  for (int m = ix-1; m > 0; m--)
  {
    cellak[m-1] = elozo[cel];
    cel = elozo[cel];
  }

  return ix;
}

int Graf::DFS(int start, std::vector<bool>& elert, int cellak[], int db)  //DFS függvény
{
  int ix = db;
  //std::cout << start << " ";                //Aktuális csúcs kiírása
  elert[start] = true;                      //Csúcs elérve (beállítás)
  cellak[ix] = start;
  ix++;

  for (int i = 0; i < v; i++)               //Gráf minden csúcsán végig
  {
    //Ha az él szomszédos és még nem volt elérve
    if (mx[start][i] == 1 && (!elert[i])) 
    {
      ix = DFS(i, elert, cellak, ix);
    }
  }
  return ix;
}

//********************************************************************************************

//Élek rendezéséhez szükséges függvény
bool compare2edges(el a, el b)          
{
  if (a.csucs1.y != b.csucs1.y)         //Ha az 1. csúcsok y koordinátái eltérnek: true <=> a < b
      return a.csucs1.y < b.csucs1.y;

  if (a.csucs1.x != b.csucs1.x)         //Ha az 1. csúcsok x koordinátái eltérnek: true <=> a < b
      return a.csucs1.x < b.csucs1.x;

  return (a.dir < b.dir);               //Ha az első csúcsok egyenlőek, az irányultság dönt
}

//********************************************************************************************

//Poligon élek rendezéséhez szükséges függvény
bool comparePoliEdges(el a, el b)          
{
  if (a.csucs1.x != b.csucs1.x)         //Ha az 1. csúcsok x koordinátái eltérnek: true <=> a < b
      return a.csucs1.x < b.csucs1.x;

  if (a.csucs2.x != b.csucs2.x)         //Ha az 2. csúcsok x koordinátái eltérnek: true <=> a < b
      return a.csucs2.x < b.csucs2.x;

  return (a.csucs1.y < b.csucs1.y);     //Az 1. csúcsok  y koordinátái döntenek
}

//********************************************************************************************

//Két csúcsok összehasonlító függvény
bool compare2nodes(geometry_msgs::Point32 a, geometry_msgs::Point32 b)
{
  if (a.x != b.x)                       //Ha az csúcsok x koordinátái eltérnek: true <=> a < b
      return a.x < b.x;

  return (a.y < b.y);                   //Ha az x koordináták egyenlőek, az y-ok döntenek
}

//********************************************************************************************

//Csúcsok közül az ismétlődőeket kivevő, majd a csúcsokat sorbarendező függvény
geometry_msgs::Point32* sort_nodes(el *elek, int el_db, int& csucs_szam)
{
  int cs_i = 0;                                     //Csúcsok indexelése                                                 
  bool nemvolt;                                     //Változó az új csúcsok felvételéhez

  //Sorbarendezett, csak egyszer előforduló csúcsokat tároló tömb
  geometry_msgs::Point32* csucsok = new geometry_msgs::Point32[el_db];

  for (int i=0; i < el_db; i++)                     //Végig a kapott éleken
  {
    nemvolt = 1;                                
    for (int j = 0; j < cs_i; j++ )                 //Keresés, hogy szerepel-e már a listában a csúcs
    {
      if (csucsok[j]==elek[i].csucs1) nemvolt = 0;                                
    }
    if (nemvolt)                                    //Ha nem szerepel, csúcs beírása a listába
    {                                 
      csucsok[cs_i] = elek[i].csucs1;
      cs_i++;
    }

    nemvolt = 1;                                    //Mindez újra, az élek második csúcsával
    for (int j = 0; j < cs_i; j++ )
    {
      if (csucsok[j]==elek[i].csucs2) nemvolt = 0;
    }
    if (nemvolt) 
    {
      csucsok[cs_i] = elek[i].csucs2;
      cs_i++;
    }
  }
  std::sort(csucsok, csucsok+el_db, compare2nodes); //Csúcsok sorbarendezése

  // for (int i=0; i < el_db; i++)                    //Sorbarendezett csúcsok kiírása
  // {
  //   ROS_INFO("cs#%d: %lf;%lf",i+1,csucsok[i].x, csucsok[i].y );
  // }
  csucs_szam = cs_i;
  return csucsok;
}

//********************************************************************************************

//Éleket előállító függvény
el makeline(geometry_msgs::Point32 cs1, geometry_msgs::Point32 cs2) 
{
  el line;
  line.csucs1 = cs1;
  line.csucs2 = cs2;
  
  if (cs2.x < cs1.x)                        //Csúcsok cseréje szükséges
    {
      line.csucs2=cs1;
      line.csucs1=cs2;
      line.dir=YPLUS;                       //Bejárandó terület az él felett
      line.mered =round(1000*(line.csucs2.y-line.csucs1.y)/(line.csucs2.x-line.csucs1.x)) / 1000;
    }
  else if (cs1.x == cs2.x)                  //Függőleges él
    {
      if (cs2.y < cs1.y)                    //Csúcsok cseréje szükséges
      {
        line.csucs2=cs1;
        line.csucs1=cs2;  
        line.dir=XMINUS;                    //Bejárandó terület az éltől balra
      }
      else
        line.dir=XPLUS;                     //Bejárandó terület az éltől jobbra

    line.mered = 100;
    }
  else
  {
    line.dir=YMINUS;                        //Bejárandó terület az él alatt
    line.mered = round(1000*(line.csucs2.y-line.csucs1.y)/(line.csucs2.x-line.csucs1.x)) / 1000;
  }
return line;
}

//********************************************************************************************

//Poligon éleit a csúcsokból előállító függvény
void poli_elkereso(el elek[], int db, geometry_msgs::Point32 cs[])
{
  for (int i = 0; i<db; i++)
  {

    int i2 = (i+1)%db;                        //Utolsó és első csúcs közti élhez index

    elek[i].csucs1 = cs[i];
    elek[i].csucs2 = cs[i2];
    
    if (elek[i].csucs2.x < elek[i].csucs1.x)
    {
    elek[i].csucs1 = cs[i2];
    elek[i].csucs2 = cs[i];
    elek[i].dir=YMINUS;                       //Poligon alsó élről lévén szó
    elek[i].mered =round(1000*(elek[i].csucs2.y-elek[i].csucs1.y)/(elek[i].csucs2.x-elek[i].csucs1.x)) / 1000;
    }
    else if (elek[i].csucs2.x  == elek[i].csucs1.x)        //Függőleges él
    {
      if (cs[i2].y < cs[i].y)                 //Csúcsok cseréje szükséges
      {
        if (i!=1 && (elek[i-1].dir==YMINUS || (elek[i-1].dir == XMINUS) && cs[i+2].x < cs[i2].x ))    // i=1 eset külön
        {
          elek[i].csucs2=cs[i];               //Fent vagyunk
          elek[i].csucs1=cs[i2];  
        }
        elek[i].dir=XPLUS;                    //Lent vagyunk
      }
      else
      {
        if (i!=1 && elek[i-1].dir==YMINUS)    // i=1 eset külön
        {
          elek[i].csucs2=cs[i];               //Fent vagyunk
          elek[i].csucs1=cs[i2];  
        }
         elek[i].dir=XMINUS;                  //Poligon területe az éltől balra
      }
    elek[i].mered = 100;
    }
    else
    {
      elek[i].dir=YPLUS;                      //Poligon területe az él alatt
      elek[i].mered =round(1000*(elek[i].csucs2.y-elek[i].csucs1.y)/(elek[i].csucs2.x-elek[i].csucs1.x)) / 1000;
    }
  }

  //Első él kezelése külön: irányultság módosítása, ha szükséges
  //Egyelőre ez nem kell!! 
  
}

//********************************************************************************************

//Trapéz cellákat előállító függvény
t_cella makecell(el alsoel, el felsoel, double x1_in, double x2_in, int &ix)
{
  t_cella cella;
  double x1 = x1_in-0.5;                //Kisebbik x koordináta
  double x2 = x2_in+0.5;                //Nagyobbik x koordináta  

  //Cella csúcspontjainak meghatározása, adatok kitöltése
  cella.cs_bal.x = x1;
  cella.cs_jobb.x = x2-0.5;
  cella.cs_bal.y = alsoel.csucs1.y+alsoel.mered*(x1-alsoel.csucs1.x);
  cella.cs_jobb.y= alsoel.csucs1.y+alsoel.mered*(x2-0.5-alsoel.csucs1.x);
  cella.m_bal = felsoel.csucs1.y + felsoel.mered*(x1-felsoel.csucs1.x)-cella.cs_bal.y;
  cella.m_jobb = felsoel.csucs1.y + felsoel.mered*(x2-0.5-felsoel.csucs1.x)-cella.cs_jobb.y;
     

  if (cella.m_jobb < 0 )                //Elfajult cella.1: háromszög, trapéz két jobb oldali csúcsa megegyezik
  {
    //Meghatározzuk azt az x koordinátát, amely esetében a kör alakú robot mindkét falat csak pontosan érinti 
    // (ez persze kisebb lesz, mint az eredeti x2 érték).

    double x2_hatar=(felsoel.csucs1.y-alsoel.csucs1.y-felsoel.mered*felsoel.csucs1.x+alsoel.mered*alsoel.csucs1.x) / 
                    (alsoel.mered-felsoel.mered);
    x2_hatar = (round(1000*x2_hatar))/1000;

    //Módosított értékek beállítása
    cella.cs_jobb.x = x2_hatar;
    cella.cs_jobb.y = alsoel.csucs1.y+alsoel.mered*(x2_hatar-alsoel.csucs1.x);
    cella.m_jobb    = 0;                //A két pont egyezik, távolságuk 0
  }
  
  if (cella.m_bal < 0 )                 //Elfajult cella.2: háromszög,trapéz két bal oldali csúcsa megegyezik
  {
    //Meghatározzuk azt az x koordinátát, amely esetében a kör alakú robot mindkét falat csak pontosan érinti 
    // (ez persze nagyobb lesz, mint az eredeti x1 érték).

    double x1_hatar=(felsoel.csucs1.y-alsoel.csucs1.y-felsoel.mered*felsoel.csucs1.x+alsoel.mered*alsoel.csucs1.x) / 
                    (alsoel.mered-felsoel.mered);
    x1_hatar = (round(1000*x1_hatar))/1000;

    //Módosított értékek beállítása
    cella.cs_bal.x = x1_hatar;
    cella.cs_bal.y = alsoel.csucs1.y+alsoel.mered*(x1_hatar-alsoel.csucs1.x);
    cella.m_bal    = 0;                //A két pont egyezik, távolságuk 0
  }

  if (cella.cs_bal==cella.cs_jobb)
  {
    t_cella zero;
    return zero;
  }
  ix++;
  return cella;
}

//********************************************************************************************

//Térkép-adat érkezett
void Robot::mapPublished_Callback( nav_msgs::OccupancyGrid terkep_be)
{ 
  if (proc_statusz != START) return;          //Feldolgozás csak egyszer
  proc_statusz = TERKEP_PUB;                  //Program következő állapotba lépett

  terkep=terkep_be;                           //Térkép elmentése
  ROS_INFO("Terkep adat erkezett.");

}

//********************************************************************************************

//Poligon adatok érkeztek
void Robot::polyPublished_Callback(costmap_converter::ObstacleArrayMsg akadaly_be)
{ 

  if (proc_statusz != POLI_FOGAD) return;     //Feldolgozás csak egyszer
  proc_statusz = TERKEP_FELDOLG;              //Kövi állapot: térkép átskálázás
  akad = akadaly_be;                          //Akadályok elmentése
  poli_db = akadaly_be.obstacles.size();      //Poligonok száma
  ROS_INFO("Osszesen %d poligon erkezett.",poli_db);
}

//********************************************************************************************

//Aktuális célpoz elérését vizsgáló függvény 
void Robot::goalReached_Callback( move_base_msgs::MoveBaseActionResult eredmeny)
{
  if (eredmeny.status.SUCCEEDED)              //Ha az aktuális célt elérte
    {
      ROS_INFO("Pozicio elerese: sikeres" );
      celbaert = 1;
    } 

  else if(eredmeny.status.ACTIVE || eredmeny.status.PENDING) //Ha még nem ért célba
    {
      ROS_INFO("Pozicio elerese: folyamatban.");
      celbaert = 0;
    }

  else                                        //Ha valami hiba történt
    ROS_INFO("Pozicio elerese elakadt, problema.");
}

//********************************************************************************************

//Kezdőpozícióba helyezve a robot
void Robot::initPoseReached_Callback( geometry_msgs::PoseWithCovarianceStamped startpoz)
{
  if (proc_statusz != INIT_POZ_FOGAD) return;     //Feldolgozás egyszer, a megfelelő pillanatban
  proc_statusz = STARTCELLA;                      //Kövi állapot: startcella megkeresése

  startpozicio = startpoz.pose.pose.position;     //Kezdőpozíció elmentése.
  celbaert = 1;                                   //Előző cél teljesítve.
  double start_x=startpozicio.x;                  //Kezdőpozíció kiírása.
  double start_y=startpozicio.y;
  ROS_INFO("Inicializalt pozicio: x=%lf y=%lf. Indulas", start_x,start_y);
}

//********************************************************************************************

//Áthaladás függvény: cella átszelése
void Robot::athaladas(int cellaidx, int utmx_idx)
{
  poli c = bcellak_k[cellaidx];                   //Aktuális poligon csúcsai kimentve külön
  geometry_msgs::Point32 p;                       //Kezdőpozíció: erre a pontra léptünk be
  p.x=ut.back().position.x;
  p.y=ut.back().position.y;
  geometry_msgs::Point32 cel = utmx[utmx_idx].ki; //Célpozíció: cella kilépési pontja

  //Virtuális cella: jelenleg nem foglalkozom vele.

  //Előkészületek
  el p_elek[c.db];
  poli_elkereso(p_elek, c.db, c.cs);              //Poligon éleinek megkeresése
  std::sort(p_elek,p_elek+c.db,comparePoliEdges); //Élek rendezése x1,x2,y1 szerint

  int mode;
  if (p.x == cel.x)       mode=0;                 //Függőleges lépésre van csak szükség
  else if (cel.x > p.x)   mode=1;                 //Áthaladás a cella jobb szélére
  else                    mode=2;                 //Áthaladás a cella bal szélére


  //Függőleges lépés a cella kilépési pontba
  if (mode==0)                                    //Lépés +y vagy -y irányba
  {
    this->addNode(cel);                           //Céépozíció beillesztés a sor végére
  }

  else
  {

    geometry_msgs::Point32 x_max_also = bszelek[cellaidx].cs_jobb;//Maximális x érték tárolása
    std::vector<int> x_alul = {};                 //Élek tárolása vektorral.
    bool elso = 1;

    for (int ii=0; ii<c.db; ii++)                 //Alsó élsorozat megállapítása
    {
      //Meredekségeket nem kell már számolni.

      //Alsó  élsorozat kezdő elemének kiszámolása
      if (p_elek[ii].csucs1.x == p_elek[0].csucs1.x && elso && 
      (p_elek[ii].dir == YPLUS || (p_elek[ii].dir==XMINUS && p_elek[ii].csucs2.y > p_elek[ii].csucs1.y ))) 
      {
        x_alul.push_back(ii);  
        elso = 0;                             
      }
      
      //Élsorozat teljes meghatározása: ha ez az él következik
      if (p_elek[ii].csucs1 != x_max_also && x_alul.size() !=0 && 
          p_elek[ii].csucs1 == p_elek[x_alul.back()].csucs2 && (p_elek[ii].dir != YMINUS)) 
      {
        x_alul.push_back(ii);                     //Következő él indexének hozzáadása a végére
      }
    }

    //ÁTHALADÁS

    switch (mode)
    {
    case 1:                                       //Áthaladás balról jobbra, az alsó élek mentén
      
      //Esetleges (lefele történő) lépés a bal oldali szélső pontba.
      this->addNode(bszelek[cellaidx].cs_bal);

      for (int i=0;i<x_alul.size(); i++)          //Végighaladás az alsó éleken
      {
        this->addNode(p_elek[x_alul[i]].csucs2);  //Új csúcsok hozzáadása az útvonalhoz.
      }
      break;

    case 2: 

      //Esetleges (lefele történő) lépés a jobb oldali szélső pontba.
      this->addNode(bszelek[cellaidx].cs_jobb);

      for (int i=x_alul.size()-1; i>=0; i--)      //Végighaladás az alsó éleken
      {
        this->addNode(p_elek[x_alul[i]].csucs1);  //Új csúcsok hozzáadása az útvonalhoz.
      }
      break;
    }

    //Esetleges függőleges lépés a cella kilépési pontjába

    this->addNode(cel);
  }
}

//********************************************************************************************

//Cellátbejár függvény: cella cikk-cakkos végigjárása
void Robot::cellatbejar(int cellaidx, int utmx_idx)
{
  poli c = bcellak_k[cellaidx];                   //Aktuális poligon csúcsai kimentve külön
  geometry_msgs::Point32 p;                       //Kezdőpozíció: erre a pontra léptünk be
  p.x=ut.back().position.x;
  p.y=ut.back().position.y;
  geometry_msgs::Point32 cel;                     //Cella bejárás végpont

  geometry_msgs::Point32 akt_cel;
  double uthossz;
  double pluszegyx, minuszegyx;

  //Előkészületek
  el p_elek[c.db];

  poli_elkereso(p_elek, c.db, c.cs);              //Poligon éleinek megkeresése  
  std::sort(p_elek,p_elek+c.db,comparePoliEdges); //Élek rendezése x1,x2,y1 szerint

  //Bejárási irány és célpozíció meghatározása
  int mod;                                        //Bejárás módja (bal -> jobb / jobb -> bal)
  if (bszelek[cellaidx].cs_jobb.x > utmx[utmx_idx].be.x)
  {
    mod = 1;                                      //BEJÁRÁS BALRÓL JOBBRA, cikkcakkozással
    cel=bszelek[cellaidx].cs_jobb;                //Célpozíció kiszámolása
  }
  else if (bszelek[cellaidx].cs_bal.x < utmx[utmx_idx].be.x)
  {
    mod = 2;                                      //BEJÁRÁS JOBBRÓL BALRA, cikkcakkozással
    cel = p_elek[0].csucs1;                       //Célpozíció kiszámolása
  }
  else mod = 0;                                 //SIMA FÜGGŐLEGES MOZGÁS


  if (mod==0)                                   //EGYSÉGNYI VÉKONY CELLA KEZELÉSE EGYBŐL
  {
    akt_cel.x = bszelek[cellaidx].cs_bal.x;
    akt_cel.y = bszelek[cellaidx].cs_bal.y + bszelek[cellaidx].m_bal;
    this->addNode(akt_cel);
    this->addNode(bszelek[cellaidx].cs_bal);
    this->addNode(utmx[utmx_idx].ki);
    return;
  }

  std::vector<int> x_alul = {};                 //Alsó és felső élek tárolása vektorral.
  std::vector<int> x_felul= {};

  geometry_msgs::Point32 x_max_also =bszelek[cellaidx].cs_jobb;
  geometry_msgs::Point32 x_max_felso =bszelek[cellaidx].cs_jobb;
  x_max_felso.y = x_max_felso.y + bszelek[cellaidx].m_jobb;
  bool elso_a = 1, elso_f = 1;

  for (int i = 0; i< c.db; i++)
  {

    //Meredekségeket már nem kell számolni

    //Alsó és felsõ élsorozatok kezdõ elemeinek kiszámolása
    if (p_elek[i].csucs1.x == p_elek[0].csucs1.x && elso_a && 
    (p_elek[i].dir == YPLUS || (p_elek[i].dir==XMINUS && p_elek[i].csucs2.y > p_elek[i].csucs1.y )))
    {
      x_alul.push_back(i);      
      elso_a =0;                        
    }
    else if (p_elek[i].csucs1.x == p_elek[0].csucs1.x && elso_f && 
    (p_elek[i].dir == YMINUS || (p_elek[i].dir==XMINUS && p_elek[i].csucs2.y < p_elek[i].csucs1.y )))
    {
      x_felul.push_back(i);
      elso_f = 0; 
    }

    //Élsorozatok teljes meghatározása

    //Ha nem elsõ elem és az él csúcsa egyezik az elõzõ alsó él végével
    if (p_elek[i].csucs1 != x_max_also && x_alul.size() !=0 && 
        p_elek[i].csucs1 == p_elek[x_alul.back()].csucs2 && (p_elek[i].dir != YMINUS)) 
    {
      x_alul.push_back(i);                      //Alsó él indexének hozzáadása
    }

    //Ha nem elsõ elem és az él csúcsa egyezik az elõzõ felsõ él végével
    if (p_elek[i].csucs1 != x_max_also && x_felul.size() !=0 && 
        p_elek[i].csucs1 == p_elek[x_felul.back()].csucs2 && (p_elek[i].dir != YPLUS) ) 
    {
      x_felul.push_back(i);                     //Felső él indexének hozzáadása
    }
  }

  //Indexek + változók: az adott oldalon hányadik élnél járunk + meredeksége
  int index_a = 0, index_f = 0;
  double m_a = p_elek[x_alul[index_a]].mered;           //ALSÓ ÉL MEREDEKSÉG
  double m_f = p_elek[x_felul[index_f]].mered;          //FELSÕ ÉL MEREDEKSÉG


  // ROS_INFO("A %d. cella elei:\n", cellaidx);
  // for (int i = 0; i < c.db; i++)
  // {
  //   std::cout<< "x1= " << p_elek[i].csucs1.x << "  y1= " << p_elek[i].csucs1.y << "   x2= " <<
  //   p_elek[i].csucs2.x << "  y2= " << p_elek[i].csucs2.y << "   dir= " << p_elek[i].dir << "  mered= " << p_elek[i].mered << "\n";
  // }
  
  // ROS_INFO("Also elsorozat:");
  // for ( int i = 0; i< x_alul.size(); i++)
  // {
  //   std::cout<< x_alul[i] <<" \t ";
  // }
  // std::cout << "\n";

  // ROS_INFO("Felso elsorozat:");
  // for ( int i = 0; i< x_felul.size(); i++)
  // {
  //   std::cout<< x_felul[i] <<"\t";
  // }
  // std::cout << "\n";

  //KONKRÉT BEJÁRÁS

  double tav_f = 0; //Aktuális felsõ él x1-étõl vett vízszintes távolság
  double tav_a = 0; //Aktuális alsó él x1-étõl vett vízszintes távolság

  switch (mod)
  {
  case 1:                                         //BALRÓL JOBBRA BEJÁRÁS 

    //STARTPONTBA HALADÁS
    this->addNode(bszelek[cellaidx].cs_bal);
    
    while (1)
    {

//FELFELE HALADÁS---------------------------------------------------------
    
    uthossz = p_elek[x_felul[index_f]].csucs1.y + m_f*tav_f -p_elek[x_alul[index_a]].csucs1.y - m_a*tav_a;
    akt_cel.x = ut.back().position.x;
    akt_cel.y = ut.back().position.y + uthossz;
    
    this->addNode(akt_cel);

    //FENT EGYSÉGNYI LÉPÉS JOBBRA

    pluszegyx = ut.back().position.x + atmero;

      //Felső élek kezelése- - - - - - - -  - - - - - - - - -  - - - - - - - 

        //Ha az aktuális felsõ él tovább tart, mint az egységnyi lépés
        if (pluszegyx < p_elek[x_felul[index_f]].csucs2.x)
        {
          akt_cel.x = pluszegyx;                  //Csak sima lépés elõre
          akt_cel.y = atmero*m_f + ut.back().position.y;
          this->addNode(akt_cel);
          tav_f = tav_f+atmero;
        }
        else                                      //Amíg az élnek hamarabb van vége, mint az egységnyi lépés
        {
          while (pluszegyx >= p_elek[x_felul[index_f]].csucs2.x)
          {
            if (index_f == x_felul.size()-1)      //Ha vége a cellának
            {
              akt_cel = p_elek[x_felul[index_f]].csucs2;
              this->addNode(akt_cel);
              break;
            }

            else if (m_f < 100)                   //Amúgy, ha a köv él nem függőleges     
            {
              akt_cel = p_elek[x_felul[index_f]].csucs2;
              this->addNode(akt_cel);
              index_f++;                          //Index növelés
              m_f=p_elek[x_felul[index_f]].mered; //Új meredekség
            }

            else                                  //Amúgy, ha a köv. él függõleges
            {
              this->addNode(p_elek[x_felul[index_f]].csucs2);
              index_f++;                          //Index növelés
              m_f=p_elek[x_felul[index_f]].mered; //Új meredekség

            }
          }

          //Ha kell, lépés az új él mentén az egységnyi hosszig
          if (pluszegyx < cel.x)
            uthossz = pluszegyx - ut.back().position.x;
          else 
            uthossz = cel.x - ut.back().position.x;

          akt_cel.x = ut.back().position.x + uthossz;
          akt_cel.y = m_f* uthossz + ut.back().position.y;
          this->addNode(akt_cel);

          //Új aktuális él "bal" csúcsától vett vízszintes távolság 
          tav_f = ut.back().position.x - p_elek[x_felul[index_f]].csucs1.x;
        }

      //ALSÓ ÉLEK KEZELÉSE - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        //Default eset: alsó él nem frissül, távolság inkrementálás
        tav_a = tav_a + atmero;
          
        //Ha (amíg) messzebb nyúlik a lépés az alsó él "végénél"
        while (p_elek[x_alul[index_a]].csucs2.x <= ut.back().position.x)
        {
          if (index_a == x_alul.size()-1)         //Ha a cella végén járunk
          {
            tav_a = ut.back().position.x - p_elek[x_alul[index_a]].csucs1.x;
            m_a = p_elek[x_alul[index_a]].mered;
            break;
          }                                       //Ha nem a cella végénél
          else if (p_elek[x_alul[index_a]].csucs2.x < ut.back().position.x )
          {
            index_a++;
            tav_a = ut.back().position.x - p_elek[x_alul[index_a]].csucs1.x;
            m_a = p_elek[x_alul[index_a]].mered;
          }
          else
          {
            index_a++;
            tav_a = ut.back().position.x - p_elek[x_alul[index_a]].csucs1.x;
            m_a = p_elek[x_alul[index_a]].mered;
            break;
          }
        }
      
//LEFELE HALADÁS--------------------------------------------------------- 

    uthossz = p_elek[x_felul[index_f]].csucs1.y + m_f*tav_f -p_elek[x_alul[index_a]].csucs1.y - m_a*tav_a;
    akt_cel.x = ut.back().position.x;
    akt_cel.y = ut.back().position.y - uthossz;
    this->addNode(akt_cel);  

    //LENT EGYSÉGNYI LÉPÉS JOBBRA---------------------------------------------

    pluszegyx = ut.back().position.x + atmero;
            
    //HA POZ == CÉL
    if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
    {
      return;                                      //Ciklusból ki, kész a cella
    }
    
    //Alsó élek kezelése- - - - - - - -  - - - - - - - - -  - - - - - - - 

        //Ha az aktuális alsó él tovább tart, mint az egységnyi lépés
        if (pluszegyx < p_elek[x_alul[index_a]].csucs2.x)
        {
          akt_cel.x = pluszegyx;                  //Csak sima lépés elõre
          akt_cel.y = atmero*m_a + ut.back().position.y;
          this->addNode(akt_cel);
          tav_a = tav_a+atmero;
        }
        else      //Amíg az élnek hamarabb van vége, mint az egységnyi lépés
        {
          int tmp = 0;
          while (pluszegyx >= p_elek[x_alul[index_a]].csucs2.x)
          {
            if (index_a == x_alul.size()-1)       //Ha vége a cellának
            {
              akt_cel = p_elek[x_alul[index_a]].csucs2;
              this->addNode(akt_cel);

              //HA POZ == CÉL
              if (cel.x == ut.back().position.x && cel.y == ut.back().position.y && p_elek[x_alul.back()].mered == 100 && tmp ==0)
              {
                //Ha a lefele mozgás után egy függőleges lépéssel a kilépési pontba kerültünk
                return;                                //Ciklusból ki, kész a cella
              }

              break;
            }

            else if (m_a < 100)                   //Amúgy, ha a köv él nem függőleges     
            {
              akt_cel.x = p_elek[x_alul[index_a]].csucs2.x;
              akt_cel.y = (akt_cel.x-ut.back().position.x)*m_a + ut.back().position.y;
              this->addNode(akt_cel);
              index_a++;                          //Index növelés
              m_a=p_elek[x_alul[index_a]].mered;  //Új meredekség
              tmp =1;
            }

            else                                  //Amúgy, ha a köv. él függõleges
            {
              this->addNode(p_elek[x_alul[index_a]].csucs2);
              index_a++;                          //Index növelés
              m_a=p_elek[x_alul[index_a]].mered;  //Új meredekség
              tmp=1;

              //HA POZ == CÉL
              if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
              {
                //Ha a lefele mozgás után egy függőleges lépéssel a kilépési pontba kerültünk
                return;                                //Ciklusból ki, kész a cella
              }
            }
          }
          //HA POZ == CÉL
          if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
          {

            //Ha lefele lépés után nem a célpontba érkeztünk és
            //Lépnünk kellett egy kicsit előre
            akt_cel.x = cel.x;
            akt_cel.y = bszelek[cellaidx].cs_jobb.y + bszelek[cellaidx].m_jobb;
            this->addNode(akt_cel);
            this->addNode(bszelek[cellaidx].cs_jobb);
            return;                                //Ciklusból ki, kész a cella
          }



          if (pluszegyx < cel.x)                  //Ha kell, lépés az új él mentén az egységnyi hosszig
            uthossz = pluszegyx - ut.back().position.x;
          else 
            uthossz = cel.x - ut.back().position.x;

          akt_cel.x = ut.back().position.x + uthossz;
          akt_cel.y = m_a* uthossz + ut.back().position.y;
          this->addNode(akt_cel);
          //Új aktuális él "bal" csúcsától vett vízszintes távolság 
          tav_a = ut.back().position.x - p_elek[x_alul[index_a]].csucs1.x;
        }
      
      //FELSŐ ÉLEK KEZELÉSE - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        //Default eset: felső él nem frissül, távolság inkrementálás
        tav_f = tav_f + atmero;
          
        //Ha (amíg) messzebb nyúlik a lépés az alsó él "végénél"
        while (p_elek[x_felul[index_f]].csucs2.x <= ut.back().position.x )
        {

          if (index_f == x_felul.size()-1)        //Ha a cella végén járunk
          {
            tav_f = ut.back().position.x - p_elek[x_felul[index_f]].csucs1.x;
            m_f = p_elek[x_felul[index_f]].mered;
            break;
          }                                       //Ha nem a cella végénél
          else if (p_elek[x_felul[index_f]].csucs2.x < ut.back().position.x )
          {
            index_f++;
            tav_f = ut.back().position.x - p_elek[x_felul[index_f]].csucs1.x;
            m_f = p_elek[x_felul[index_f]].mered;
          }
          else 
          {
            index_f++;
            tav_f = ut.back().position.x - p_elek[x_felul[index_f]].csucs1.x;
            m_f = p_elek[x_felul[index_f]].mered;
            break;
          }
        }
    }

    // for (int i=0; i < ut.size(); i++)
    // {
    //   std::cout << ut[i].position.x << ";" << ut[i].position.y << "\n" ;
    // }

    break;

  case 2:

    tav_a =0;
    tav_f =0;
    std::vector<int> seged1 = x_alul;             //Élsorszámok visszafele
    std::vector<int> seged2 = x_felul;

    for (int k = 0; k< x_alul.size(); k++)
      x_alul[k] = seged1[x_alul.size()-1-k];

    for (int k = 0; k< x_felul.size(); k++)
      x_felul[k] = seged2[x_felul.size()-1-k];

    m_a = p_elek[x_alul[index_a]].mered;          //Alsó (jobb szélső) él meredekség
    m_f = p_elek[x_felul[index_f]].mered;         //Felső (jobb szélső) él meredekség
    
    //STARTPONTBA HALADÁS
    this->addNode(bszelek[cellaidx].cs_jobb);
    
    
    while (1)
    {

//FELFELE HALADÁS---------------------------------------------------------

    uthossz= p_elek[x_felul[index_f]].csucs2.y-m_f*tav_f-p_elek[x_alul[index_a]].csucs2.y + m_a*tav_a; 
    akt_cel.x = ut.back().position.x;
    akt_cel.y = ut.back().position.y + uthossz;
    this->addNode(akt_cel);
                        
    //FENT EGYSÉGNYI LÉPÉS BALRA--------------------------------------------- 

    minuszegyx = ut.back().position.x - atmero;
            
      //Felső élek kezelése- - - - - - - - - - - - - - - - - - - - - - - - - 
            
        //Ha az aktuális felsõ él tovább tart, mint az egységnyi lépés
        if (minuszegyx > p_elek[x_felul[index_f]].csucs1.x)
        {
          akt_cel.x = minuszegyx;                 //Csak sima lépés elõre
          akt_cel.y = ut.back().position.y - atmero*m_f;
          this->addNode(akt_cel);
          tav_f = tav_f+atmero;
        }
        else
        {                                         //Amíg az élnek hamarabb van vége, mint az egységnyi lépés
          while (minuszegyx <= p_elek[x_felul[index_f]].csucs1.x)
          {
            if (index_f == x_felul.size()-1)      //Ha vége a cellának
            {
              akt_cel = p_elek[x_felul[index_f]].csucs1;
              this->addNode(akt_cel);
              break;
            }

            else if (m_f < 100)                   //Amúgy, ha a köv él nem függőleges     
            {
              akt_cel = p_elek[x_felul[index_f]].csucs1;
              this->addNode(akt_cel);
              index_f++;                          //Index növelés
              m_f=p_elek[x_felul[index_f]].mered; //Új meredekség
            }

            else                                  //Amúgy, ha a köv. él függõleges
            {
              this->addNode(p_elek[x_felul[index_f]].csucs1);
              index_f++;                          //Index növelés
              m_f=p_elek[x_felul[index_f]].mered; //Új meredekség
            }
          }

          //Ha kell, lépés az új él mentén az egységnyi hosszig
          if (minuszegyx > cel.x)
            uthossz = ut.back().position.x - minuszegyx;
          else 
            uthossz = ut.back().position.x - cel.x;

          akt_cel.x = ut.back().position.x - uthossz;
          akt_cel.y = ut.back().position.y - m_f * uthossz;
          this->addNode(akt_cel);


          //Új aktuális él "jobb" csúcsától vett vízszintes távolság 
          tav_f = p_elek[x_felul[index_f]].csucs2.x - ut.back().position.x;
        }

      //ALSÓ ÉLEK KEZELÉSE - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        //Default eset: alsó él nem frissül, távolság inkrementálás
        tav_a = tav_a + atmero;

        //Ha (amíg) messzebb nyúlik a lépés az alsó él "végénél"
        while (p_elek[x_alul[index_a]].csucs1.x >= ut.back().position.x)
        {
          if (index_a == x_alul.size()-1)         //Ha a cella végén járunk
          {
            tav_a = p_elek[x_alul[index_a]].csucs2.x-ut.back().position.x;
            m_a = p_elek[x_alul[index_a]].mered;
            break;
          }                                       //Ha nem a cella végénél
          else if (p_elek[x_alul[index_a]].csucs1.x > ut.back().position.x )
          {
            index_a++;
            tav_a = p_elek[x_alul[index_a]].csucs2.x-ut.back().position.x;
            m_a = p_elek[x_alul[index_a]].mered;
          }
          else
          {
            index_a++;
            tav_a = p_elek[x_alul[index_a]].csucs2.x-ut.back().position.x;
            m_a = p_elek[x_alul[index_a]].mered;
            break;
          }
        }

//LEFELE HALADÁS--------------------------------------------------------- 

    uthossz = p_elek[x_felul[index_f]].csucs2.y - m_f*tav_f -p_elek[x_alul[index_a]].csucs2.y + m_a*tav_a;
    akt_cel.x = ut.back().position.x;
    akt_cel.y = ut.back().position.y - uthossz;
    this->addNode(akt_cel);  

    //LENT EGYSÉGNYI LÉPÉS BALRA----------------------------------------------

    minuszegyx = ut.back().position.x - atmero;
            
    //HA POZ == CÉL
    if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
    {
      return;                                      //Ciklusból ki, kész a cella
    }

    //Alsó élek kezelése- - - - - - - -  - - - - - - - - -  - - - - - - - 

        //Ha az aktuális alsó él tovább tart, mint az egységnyi lépés
        if (minuszegyx > p_elek[x_alul[index_a]].csucs1.x)
        {
          akt_cel.x = minuszegyx;                  //Csak sima lépés elõre
          akt_cel.y = ut.back().position.y-atmero*m_a;
          this->addNode(akt_cel);
          tav_a = tav_a+atmero;
        }
        else      //Amíg az élnek hamarabb van vége, mint az egységnyi lépés
        {
          int tmp = 0;
          while (minuszegyx <= p_elek[x_alul[index_a]].csucs1.x)
          {
            if (index_a == x_alul.size()-1)       //Ha vége a cellának
            {
              akt_cel = p_elek[x_alul[index_a]].csucs1;
              this->addNode(akt_cel);

              //HA POZ == CÉL
              if (cel.x == ut.back().position.x && cel.y == ut.back().position.y && p_elek[x_alul.back()].mered == 100 && tmp==0)
              {
                //Ha a lefele mozgás után egy függőleges lépéssel a kilépési pontba kerültünk
                return;                                //Ciklusból ki, kész a cella
              }

              break;
            }

            else if (m_a < 100)                   //Amúgy, ha a köv él nem függőleges     
            {
              akt_cel.x = p_elek[x_alul[index_a]].csucs1.x;
              akt_cel.y = ut.back().position.y + (akt_cel.x-ut.back().position.x)*m_a ;
              this->addNode(akt_cel);
              index_a++;                          //Index növelés
              m_a=p_elek[x_alul[index_a]].mered;  //Új meredekség
              tmp = 1;
            }

            else                                  //Amúgy, ha a köv. él függõleges
            {
              this->addNode(p_elek[x_alul[index_a]].csucs1);
              index_a++;                          //Index növelés
              m_a=p_elek[x_alul[index_a]].mered;  //Új meredekség
              tmp = 1;

              //HA POZ == CÉL
              if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
              {
                return;                                //Ciklusból ki, kész a cella
              }
            }
          }

          //HA POZ == CÉL
          if (cel.x == ut.back().position.x && cel.y == ut.back().position.y)
          {

            //Ha lefele lépés után nem a célpontba érkeztünk és
            //Lépnünk kellett egy kicsit előre
            akt_cel.x = cel.x;
            akt_cel.y = bszelek[cellaidx].cs_bal.y + bszelek[cellaidx].m_bal;
            this->addNode(akt_cel);
            this->addNode(bszelek[cellaidx].cs_bal);
            return;                                //Ciklusból ki, kész a cella
          }

          if (minuszegyx > cel.x)                  //Ha kell, lépés az új él mentén az egységnyi hosszig
            uthossz = ut.back().position.x - minuszegyx;
          else 
            uthossz = ut.back().position.x - cel.x;

          akt_cel.x = ut.back().position.x - uthossz;
          akt_cel.y = ut.back().position.y - m_a * uthossz;
          this->addNode(akt_cel);
          //Új aktuális él "jonn" csúcsától vett vízszintes távolság 
          tav_a = p_elek[x_alul[index_a]].csucs2.x-ut.back().position.x;
        }
      
      //FELSŐ ÉLEK KEZELÉSE - - - - - - - - - - - - - - - - - - - - - - - - - 
        
        //Default eset: felső él nem frissül, távolság inkrementálás
        tav_f = tav_f + atmero;
          
        //Ha (amíg) messzebb nyúlik a lépés az alsó él "végénél"
        while (p_elek[x_felul[index_f]].csucs1.x >= ut.back().position.x )
        {

          if (index_f == x_felul.size()-1)        //Ha a cella végén járunk
          {
            tav_f =p_elek[x_felul[index_f]].csucs2.x - ut.back().position.x;
            m_f = p_elek[x_felul[index_f]].mered;
            break;
          }                                       //Ha nem a cella végénél
          else if (p_elek[x_felul[index_f]].csucs1.x > ut.back().position.x )
          {
            index_f++;
            tav_f = p_elek[x_felul[index_f]].csucs2.x - ut.back().position.x;
            m_f = p_elek[x_felul[index_f]].mered;
          }
          else
          {
            index_f++;
            tav_f = p_elek[x_felul[index_f]].csucs2.x - ut.back().position.x;
            m_f = p_elek[x_felul[index_f]].mered;
            break;
          }
        }
    }
    break;
  }

  // for (int i=0; i < ut.size(); i++)
  // {
  //   std::cout << ut[i].position.x << ";" << ut[i].position.y << "\n" ;
  // }
}                                                   

//********************************************************************************************

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pub_goal");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);

  Robot robot1;                                   //Robot objektum létrehozás
  geometry_msgs::PoseStamped aktpoz;              //Aktuális pozíció tárolása
  geometry_msgs::PoseStamped goal;                //Aktuális célpozíció tárolása
  visualization_msgs::Marker utvonal;             //Már megtett útvonal kirajzolása

  {                                               //Mindenféle inicializálása az útvonalnak
    utvonal.header.frame_id = "map";
    utvonal.header.stamp = ros::Time::now();
    utvonal.ns = "pub_goal";
    utvonal.action = visualization_msgs::Marker::ADD;
    utvonal.type = visualization_msgs::Marker::LINE_LIST;
    utvonal.pose.orientation.w = 1.0;
    utvonal.id = 1;
    utvonal.scale.x = 0.05;
    utvonal.color.a = 1;
    utvonal.color.g = 0.5;
    utvonal.color.b = 0.4;
    utvonal.color.r = 0.9;
  }

    int elszam      = 0;                            //GRID alapú élek száma
    int cell_ix     = 0;                            //Trapéz cellák száma
    int b_szam      = 0;                            //Boustrophedon cellák száma
    int ut_szam     = 0;                            //Útmátrix mérete
    int startcella  =-1;                            //Azon cella sorszáma, amelyen belül van a robot
    int index       = 1;                            //Elért célpozíciók számolása
  
    //Célpozíció kiküldése
    ros::Publisher pub_goal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
    //Térkép adatok kiküldése
    ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map",20);
    //Vizualizíció: megtett útvonal kirajzolása
    ros::Publisher pub_marker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //Frissített térkép-adatok
    ros::Publisher pub_map_update = nh.advertise<nav_msgs::OccupancyGrid>("/map_updates",20);
    //Pozíció-inicializálás érzékelése
    ros::Subscriber sub_pos = nh.subscribe("/initialpose", 1000, &Robot::initPoseReached_Callback, &robot1);
    //Célpozíció elérésének érzékelése
    ros::Subscriber sub_goal = nh.subscribe("/move_base/result", 1000, &Robot::goalReached_Callback,&robot1);
    //Térkép adatok fogadása
    ros::Subscriber sub_map = nh.subscribe("/map", 10000, &Robot::mapPublished_Callback, &robot1);
    //Térkép alapján képzett poligonok fogadása
    ros::Subscriber sub_poly = nh.subscribe("/costmap_converter/costmap_obstacles", 10000, &Robot::polyPublished_Callback, &robot1);
  
  while (ros::ok())
  {

//Kapott (eredeti) térkép publikálása---------------------------------------------------------
  if (robot1.proc_statusz==TERKEP_PUB)
    {

      robot1.proc_statusz = TERKEP_FELDOLG;           //Tovább a program következő szakaszára
      pub_map.publish(robot1.terkep);             //Publikálás a costmap-converter számára
      ROS_INFO("Terkep kikuldve a costmap converternek");
    }

//Térkép publikálása costmap_converter számára, grid térkép átméretezés-----------------------
  if (robot1.proc_statusz == TERKEP_FELDOLG)
    {

      robot1.proc_statusz = CMAP_ELEK;            //Feldolgozás mindössze egy alkalommal
      ROS_INFO("Terkep adatok...");               //Kapott térkép adatainak kiírása a terminálba.
      //ROS_INFO("Felbontas: %lf", robot1.terkep.info.resolution);
      //ROS_INFO("Meret: %u x %u", robot1.terkep.info.height, robot1.terkep.info.width);
      //ROS_INFO("Origo: (%lf;%lf)", robot1.terkep.info.origin.position.x, robot1.terkep.info.origin.position.y);
      //ROS_INFO("Adatmennyiseg: %lu", robot1.terkep.data.size());

      //robot1.dilatacio();                         //Dilatáció az eredeti térképen!!

      
      double h_regi = robot1.terkep.info.height;  //Osztáshoz szükséges a double típus.
      double w_regi = robot1.terkep.info.width;   //Osztáshoz szükséges a double típus.

      //Átskálázott rácsos térkép paramétereinek meghatározása.
      robot1.skala = ceil(robot1.atmero/robot1.terkep.info.resolution);   //Skála
      robot1.terkep_uj.info.resolution = robot1.terkep.info.resolution * robot1.skala;
      robot1.terkep_uj.info.height = ceil(h_regi / robot1.skala);         //Magasság (db pixel)
      robot1.terkep_uj.info.width = ceil(w_regi / robot1.skala);          //Szélesség (db pixel)
      robot1.terkep_uj.info.origin = robot1.terkep.info.origin;           //Referenciapont
      robot1.terkep_uj.header.stamp = ros::Time::now();                   //Időbélyeg
      int8_t terkepadat[(robot1.terkep_uj.info.height*robot1.terkep_uj.info.width)]={}; //Adatok
      

      //ROS_INFO("Skala a ket terkep felbontasa kozt: %d", robot1.skala);   //Új térkép adatainak kiírása.
      //ROS_INFO("Uj meret: %u x %u", robot1.terkep_uj.info.height, robot1.terkep_uj.info.width);
      //unsigned long int ta = sizeof(terkepadat);
      //ROS_INFO("Uj adatmennyiseg: %lu", ta); 

      int h = robot1.terkep.info.height;          //Eredeti térkép szélessége, magassága
      int w = robot1.terkep.info.width;           //rövidebb névvel illetve.
      short idx = 0;                              //Új térkép adatainak indexe.
      int obst = 0;                               //Logikai változó: van-e akadály

      //Kernelek átlagolása, kisebb felbontású térkép előállítása
      for (int i=0; i<(robot1.terkep.data.size()-((w-1)%robot1.skala)-((h-1)%robot1.skala)*w); i=i+robot1.skala)
      {
        obst=0;                                             //Változó lenullázása
        for (int j = 0; j < robot1.skala; j++)              //Lépés a következő sorokra
        {
          if (i+j*w > robot1.terkep.data.size() ) break;    //Ha nincs több sor, vége
          for (int k = 0; k < robot1.skala; k++)            //Lépés a sor következő elemére
          {
            if ((i+k)%w ==0 ) break;                        //Ha a sor végére értünk, kilépés
            if (robot1.terkep.data[i+j*w+k]>0) obst++;      //Pixel vizsgálata 
            //obst = obst || (robot1.terkep.data[i+j*w+k]>0);        
          }
        }

        if (obst > 0)                                        //Új adathalmaz kitöltése: 
          terkepadat[idx]= char(100);                        //hangolható, mikor vege fel akadálynak
        else 
          terkepadat[idx]= 0;

        //std::cout << int(terkepadat[idx]) << "; ";        //Új adatok kiírása egyesével

        if ((i+robot1.skala)%w < robot1.skala)              //Eredeti kép index korrigálása, ha kell
          i = i+(robot1.skala-1)*w+-(i+robot1.skala)%w;
        
        idx = idx+1;                                        //Új kép index léptetése
      }
    
      std::vector<signed char> t_vektor(terkepadat, terkepadat+sizeof(terkepadat));
      robot1.terkep_uj.data = t_vektor;                     //Új térkép objketumhoz adatok hozzáadása
      pub_map_update.publish(robot1.terkep_uj);
    }

//Costmap converteres oligon adatok feldolgozása, élkeresés----------------------------------- 
  if (robot1.proc_statusz == CMAP_ELEK)  
    {

      robot1.proc_statusz = GRID_ELKERESES;                 //Következő szakaszra ugrás
    
      int elszam = 0;                                       //Összes él számláló
      int akt_meret = 0;                                    //Aktuális poligon élszáma
      int ix =0 ;                                           //Index  
      geometry_msgs::Polygon poligonok[robot1.poli_db];     //Poligonokat tároló tömb létrehozása

      for (int i = 0; i< robot1.poli_db; i++)               //Poligonok elmentése egy tömbbe.
      {
          poligonok[i] = robot1.akad.obstacles[i].polygon;
      }

      for (int j = 0; j<robot1.poli_db;j++)                 //Végig a poligonokon...
      {
        elszam = elszam+poligonok[j].points.size()-1;
      }
      el cmap_elek[elszam];                                 //Éleket tároló tömb létrehozása

      for (int j = 0; j<robot1.poli_db;j++)                 //Végig a poligonokon
      {
        akt_meret = poligonok[j].points.size();             //Poligon csúcsainak száma

        for (int jj = 0; jj< akt_meret-1; jj++)             //Végig a poligon csúcsain
        { 
          //Éleket előállító függvény
          cmap_elek[ix] = makeline(poligonok[j].points[jj], poligonok[j].points[((jj+1)%akt_meret)]);
          ix++;                                             //Index növelés    
        }  
      }

      std::sort(cmap_elek, cmap_elek+elszam, compare2edges);//Élek rendezése a megfelelő módon

      ROS_INFO("Kozben a poligonok feldolgozasa, a costmap alapu elek sorbarendezese kesz!");
      //ROS_INFO("%d darab el szuletett.", elszam);
      /* for (ix=0;ix < elszam; ix++)                          //Costmap élek kiírása
        {   
          ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",ix, cmap_elek[ix].csucs1.x, cmap_elek[ix].csucs1.y,
            cmap_elek[ix].csucs2.x, cmap_elek[ix].csucs2.y, cmap_elek[ix].dir, cmap_elek[ix].mered);
        }*/   
    }

// Grid alapján élek keresése-----------------------------------------------------------------
  if (robot1.proc_statusz == GRID_ELKERESES) 
    {
      robot1.proc_statusz = TRAPEZ_CELLAK;                      //Lépés a következő szakaszba           

      el *grid_el_tomb = new el[300];                           //Élek tömbjének létrehozása
      elszam =0;                                                //Élek indexelése

      geometry_msgs::Point32 origo;                             //Koordináta, amihez igazodunk
      origo.x = robot1.terkep_uj.info.origin.position.x;        
      origo.y = robot1.terkep_uj.info.origin.position.y;
      double egyseg = robot1.terkep_uj.info.resolution;         //Egy négyzetrács mérete

      geometry_msgs::Point32 bal_1, jobb_1, bal_2, jobb_2;      //Csúcsok az élek előállításához
      cellapar c_jobb_regi, c_jobb_uj, c_bal_regi, c_bal_uj;    //Cella-párok értékének tárolása
      bool c_akt, c_jobb, c_bal;                                //Aktuális és a 2 szomszédos csúcs értéke
      int h_uj = robot1.terkep_uj.info.height;                  //Vízszintes cellaszám
      int w_uj = robot1.terkep_uj.info.width;                   //Függőleges cellaszám
      ROS_INFO("Kovetkezzenek a grid alapu elek!");

      //Vízszintes élek vizsgálata
      for (int i = 0;i < h_uj; i++)                             //Végig minden soron
      {
        c_bal_regi = F_F;                                       //Kezdőérték: foglalt cellák
        c_jobb_regi = F_F;

        for (int j = 0; j <= w_uj; j++)                         //Végig egy vízszintes sor elemein
        {

          if (j == w_uj) c_akt = 1;
          else c_akt = (robot1.terkep_uj.data[i*h_uj+j] > 0);   //Aktuális cella értéke
          if (i ==0 || j==w_uj) c_jobb =1;
          else c_jobb=(robot1.terkep_uj.data[i*h_uj+j-w_uj]>0); //Jobbra lévő cella értéke
          if (i == h_uj-1 || j == w_uj) c_bal = 1;
          else c_bal=(robot1.terkep_uj.data[i*h_uj+j+w_uj]>0);  //Balra lévő cella értéke
          
          if (c_akt && c_jobb) c_jobb_uj = F_F;                 //Új cellapár-értékek állítása
          if (c_akt && !c_jobb) c_jobb_uj = F_SZ;               //jobbra
          if (!c_akt && c_jobb) c_jobb_uj = SZ_F;
          if (!c_akt && !c_jobb) c_jobb_uj = SZ_SZ;
          if (c_akt && c_bal) c_bal_uj = F_F;                   //balra
          if (c_akt && !c_bal) c_bal_uj = F_SZ;
          if (!c_akt && c_bal) c_bal_uj = SZ_F;
          if (!c_akt && !c_bal) c_bal_uj = SZ_SZ;

          if (c_jobb_regi != SZ_F && c_jobb_uj == SZ_F)         //Ha jobbra új él kezdődik
          {
            jobb_1.x = origo.x + j*egyseg;                      //Jobb oldali kezdőcsúcs beállítása
            jobb_1.y = origo.y + i*egyseg;
          }

          if (c_bal_regi != SZ_F && c_bal_uj == SZ_F)           //Ha balra új él kezdődik
          {
            bal_1.x = origo.x + j*egyseg;                       //Bal oldali kezdőcsúcs beállítása
            bal_1.y = origo.y + (i+1)*egyseg;
          }
        
          if (c_jobb_regi == SZ_F && c_jobb_uj != SZ_F)         //Ha jobbra él zárul
          {
            jobb_2.x = origo.x + j*egyseg;                      //Zárócsúcs beállítás
            jobb_2.y = origo.y + i*egyseg;
            grid_el_tomb[elszam] = makeline(jobb_2, jobb_1);    //Él hozzáadás a tömb következő helyére

            //ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",elszam,
            //grid_el_tomb[elszam].csucs1.x,grid_el_tomb[elszam].csucs1.y,grid_el_tomb[elszam].csucs2.x,
            //grid_el_tomb[elszam].csucs2.y,grid_el_tomb[elszam].dir, grid_el_tomb[elszam].mered);
            elszam++;
          }

          if (c_bal_regi == SZ_F && c_bal_uj != SZ_F)           //Ha balra él zárul
          {
            bal_2.x = origo.x + j*egyseg;                       //Zárócsúcs megadás
            bal_2.y = origo.y + (i+1)*egyseg;
            grid_el_tomb[elszam] = makeline(bal_1, bal_2);      //Él hozzáadás (megfelelő irány)

            //ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",elszam,
            //grid_el_tomb[elszam].csucs1.x,grid_el_tomb[elszam].csucs1.y,grid_el_tomb[elszam].csucs2.x,
            //grid_el_tomb[elszam].csucs2.y,grid_el_tomb[elszam].dir, grid_el_tomb[elszam].mered);
            elszam++;
          }


          c_jobb_regi = c_jobb_uj;                              //Lépés tovább: régi cellapárok 
          c_bal_regi = c_bal_uj;                                //frissítése
        }
      }
//-------------------------------------------------------------------------------------------

      //Függőleges élek vizsgálata
      for (int i = 0;i < w_uj; i++)                             //Végig minden soron
      {
        c_bal_regi = F_F;                                       //Kezdőérték: foglalt cellák
        c_jobb_regi = F_F;

        for (int j = 0; j <= h_uj; j++)                         //Végig egy vízszintes sor elemein
        {

          if (j == h_uj) c_akt = 1;
          else c_akt = (robot1.terkep_uj.data[j*w_uj+i] > 0);   //Aktuális cella értéke
          if (i == w_uj-1 || j == h_uj) c_jobb =1;
          else c_jobb=(robot1.terkep_uj.data[j*w_uj+i+1]>0);    //Jobbra lévő cella értéke
          if (i == 0 || j == h_uj) c_bal = 1;
          else c_bal=(robot1.terkep_uj.data[j*w_uj+i-1]>0);     //Balra lévő cella értéke
          
          if (c_akt && c_jobb) c_jobb_uj = F_F;                 //Új cellapár-értékek állítása
          if (c_akt && !c_jobb) c_jobb_uj = F_SZ;               //jobbra
          if (!c_akt && c_jobb) c_jobb_uj = SZ_F;
          if (!c_akt && !c_jobb) c_jobb_uj = SZ_SZ;
          if (c_akt && c_bal) c_bal_uj = F_F;                   //balra
          if (c_akt && !c_bal) c_bal_uj = F_SZ;
          if (!c_akt && c_bal) c_bal_uj = SZ_F;
          if (!c_akt && !c_bal) c_bal_uj = SZ_SZ;

          if (c_jobb_regi != SZ_F && c_jobb_uj == SZ_F)         //Ha jobbra új él kezdődik
          {
            jobb_1.x = origo.x + (i+1)*egyseg;                  //Jobb oldali kezdőcsúcs beállítása
            jobb_1.y = origo.y + j*egyseg;
          }

          if (c_bal_regi != SZ_F && c_bal_uj == SZ_F)           //Ha balra új él kezdődik
          {
            bal_1.x = origo.x + i*egyseg;                       //Bal oldali kezdőcsúcs beállítása
            bal_1.y = origo.y + j*egyseg;
          }
        
          if (c_jobb_regi == SZ_F && c_jobb_uj != SZ_F)         //Ha jobbra él zárul
          {
            jobb_2.x = origo.x + (i+1)*egyseg;                  //Zárócsúcs beállítás
            jobb_2.y = origo.y + j*egyseg;
            grid_el_tomb[elszam] = makeline(jobb_2, jobb_1);    //Él hozzáadás a tömb következő helyére

            //ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",elszam,
            //grid_el_tomb[elszam].csucs1.x,grid_el_tomb[elszam].csucs1.y,grid_el_tomb[elszam].csucs2.x,
            // grid_el_tomb[elszam].csucs2.y,grid_el_tomb[elszam].dir, grid_el_tomb[elszam].mered);
            elszam++;
          }

          if (c_bal_regi == SZ_F && c_bal_uj != SZ_F)           //Ha balra él zárul
          {
            bal_2.x = origo.x + i*egyseg;                       //Zárócsúcs megadás
            bal_2.y = origo.y + j*egyseg;
            grid_el_tomb[elszam] = makeline(bal_1, bal_2);      //Él hozzáadás (megfelelő irány)

            // ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",elszam,
            //grid_el_tomb[elszam].csucs1.x,grid_el_tomb[elszam].csucs1.y,grid_el_tomb[elszam].csucs2.x,
            // grid_el_tomb[elszam].csucs2.y,grid_el_tomb[elszam].dir, grid_el_tomb[elszam].mered);
            elszam++;
          }

          c_jobb_regi = c_jobb_uj;                              //Lépés tovább: régi cellapárok 
          c_bal_regi = c_bal_uj;                                //frissítése
        }
      }

      robot1.grid_elek = new el[elszam];                        //Megfelelő méretű dinamikus tömb
      std::copy(grid_el_tomb, grid_el_tomb+elszam, robot1.grid_elek);     //Él adatok másolása
      std::sort(robot1.grid_elek, robot1.grid_elek+elszam, compare2edges);//Élek rendezése
      delete[] grid_el_tomb;                                   //Nagy dinamikus tömb törlés
      ROS_INFO("Elek felvetele megtortent. Osszesen %d el keletkezett.", elszam);
      /*for (int i = 0; i<elszam; i++)                            //Kapott élek kiírása
      {
        ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",i, robot1.grid_elek[i].csucs1.x, 
        robot1.grid_elek[i].csucs1.y,robot1.grid_elek[i].csucs2.x, robot1.grid_elek[i].csucs2.y,
        robot1.grid_elek[i].dir, robot1.grid_elek[i].mered);
      }*/
    }

//Élek alapján trapéz cellákra osztás---------------------------------------------------------
  if (robot1.proc_statusz == TRAPEZ_CELLAK )
    {
      robot1.proc_statusz = BOUSTROPHEDON;

      t_cella *tcella_tomb = new t_cella[200];    //Trapéz cellák tömbjének létrehozása
      cell_ix =0;                                 //Trapéz cellák indexelése
      
      esemenyek esemeny = DEF; 
      int csucsszam; 
      geometry_msgs::Point32* csucsok = sort_nodes( robot1.grid_elek, elszam, csucsszam);
      int c,d;
      double m_c, m_d;
      elsorszamtomb J;                            //Élsorszámokat tartalmazó tömb
      cellakezdet K;                              //cellakezdeteket tartalmazó tömb                             
     

      //Végighaladunk a sorbarendezett csúcsokon
      for (int index = 0; index <csucsszam;index++)
      {
        double line = csucsok[index].x;

        for (c = 0; c< elszam-1; c++)             //Végigmegyünk az éleken 
        {
          if (robot1.grid_elek[c].csucs1==csucsok[index])   //Ha van a vonalon csúcs, amibõl INDUL ÉL
          {
            for (d = c+1; d<elszam; d++)          //Megkeressük a csúcshoz tartozó másik élt
            {
              if (robot1.grid_elek[c].csucs1==robot1.grid_elek[d].csucs1) //Ha a másik él is induló él
              {
                esemeny = NYIT;
                break;
              }
              if (robot1.grid_elek[c].csucs1==robot1.grid_elek[d].csucs2) //Ha a másik él záródó él
              {
                esemeny = FOLY;
                break;
              }
            }
          }

          if (esemeny==NYIT)                      //Ha a csúcsból új akadály NYÍLIK
          {
            esemeny = DEF;                        //Esemény lekezelve, lenullázás
            m_c = robot1.grid_elek[c].mered;      //c. él meredeksége
            m_d = robot1.grid_elek[d].mered;      //d. él meredeksége

            if (J.size == 0)                      //ELSÕ 2 él hozzáadása (ezután J nem lesz soha 0)
            {
              J.temp = new int[2];
              J.size = 2;
              if (m_c > m_d) 
              {
                J.temp[0]=d;
                J.temp[1]=c;
              }
              else                                //A lejjebb irányuló, kisebb meredekségû lesz "alul"
              {
                J.temp[0]=c;
                J.temp[1]=d;
              }
              J.akt = J.temp;
              K.temp = new double[2];
              K.regi = new double[1];
              K.temp[0]=line+0.5;
              K.temp[1]=line+0.5;
              K.uj = K.temp;
              K.uj_size = 2;
            }
            else                                  //Ha már a területen belül vagyunk (terület alja+teteje "nyitva")
            {
              delete[] K.regi; 
              K.regi = K.uj;
              K.regi_size = K.uj_size;
              int x = 0;
              double y_akt = -10000;
              bool felulre = 0;
              //Végigmegyünk a nyitott éleken és a megfelelõ helyre illesztjük a két újat
              while (y_akt < robot1.grid_elek[c].csucs1.y)
              {
                if (x == J.size) 
                {
                  felulre = 1;
                  break;
                }
                x++;                              //Következõ él és a line metszéspontjának kiszámolása
                double m_akt = robot1.grid_elek[J.akt[x-1]].mered;
                y_akt = robot1.grid_elek[J.akt[x-1]].csucs1.y + m_akt*(line-robot1.grid_elek[J.akt[x-1]].csucs1.x);
              }

              int e_felso, e_also;
              if (m_c>m_d)                        //Új élek beillesztésének sorrendje a két meredekségtõl függ
              {
                e_felso = c;
                e_also = d;
              }
              else 
              {
                e_felso = d;
                e_also = c;
              }

              J.temp = new int[J.size+2];
              
              if (felulre)                        //Az összes nyitott él fölé kell illeszteni
              {
                std::copy(J.akt, J.akt+J.size, J.temp); 
                J.temp[J.size] = e_also;
                J.temp[J.size+1] = e_felso;
              }
              else if (x == 1)                    //Az összes nyitott él alá kell illeszteni
              {
                J.temp[0] = e_also;
                J.temp[1] = e_felso;
                std::copy(J.akt, J.akt+J.size, J.temp+2);
              }
              else                                //Új élek hozzáadása
              {
                std::copy(J.akt, J.akt+x-1, J.temp);
                J.temp[x-1] = e_also;
                J.temp[x] = e_felso;
                std::copy(J.akt+x-1, J.akt+J.size, J.temp+x+1);
              }

              delete[] J.akt;  
              J.akt = J.temp;
              J.temp = nullptr;
              J.size=J.size+2;

              //Ha a két új él az akadály "szétnyílása"
              if (robot1.grid_elek[e_also].dir==YPLUS && ( robot1.grid_elek[e_felso].dir == YMINUS ||
                  robot1.grid_elek[e_felso].dir == XPLUS ))
              {
                K.temp = new double[(K.uj_size + 2)];

                if (felulre)                      //Cellakezdetek módosítása
                {
                  std::copy(K.uj, K.uj+K.uj_size, K.temp);
                  K.temp[K.uj_size]= line+0.5;
                  K.temp[K.uj_size+1]= line+0.5;
                }
                else 
                {
                  std::copy(K.uj, K.uj+x-1, K.temp);
                  K.temp[x-1] = line+0.5;
                  K.temp[x] = line+0.5;
                  std::copy(K.uj+x-1, K.uj+K.uj_size, K.temp+x+1);
                }
                K.uj = K.temp;
                K.temp = nullptr;
                K.uj_size=K.uj_size+2;
              }
              else                                //Ellenkező esetben
              {
                K.temp = new double[K.uj_size + 2];
                std::copy(K.uj, K.uj+x-2, K.temp);
                K.temp[x-2] = line+0.5;
                K.temp[x-1] = line+0.5;
                K.temp[x] = line+0.5;
                K.temp[x+1] = line+0.5;
                std::copy(K.uj+x, K.uj+K.uj_size, K.temp+x+2);
                K.uj = K.temp;
                K.temp = nullptr;
                K.uj_size=K.uj_size+2;
              }

              if (memcmp(K.regi, K.uj, 8*K.uj_size) != 0  &&  K.regi[x-1] <= line+0.5)
              {
                if (robot1.grid_elek[e_also].dir==YMINUS && (robot1.grid_elek[e_felso].dir ==YPLUS ||
                robot1.grid_elek[e_felso].dir ==XMINUS))
                {
                  //ÚJ CELLA KÖVETKEZIK: KÉSZ
                  tcella_tomb[cell_ix]=makecell(robot1.grid_elek[J.akt[x-2]],robot1.grid_elek[J.akt[x+1]],K.regi[x-1],line, cell_ix);
                }
              }
            }
            // std::cout << "NYITAS J: \n";
            // for (int ii=0; ii < J.size; ii++)  std::cout << J.akt[ii] << "; ";
            // for (int ii=0; ii < K.uj_size; ii++)  std::cout << K.uj[ii] << "; ";
            // std::cout << " \n";
          }

          if (esemeny==FOLY)                      //Ha a csúcsban egy él zárul, egy él nyílik
          {
            esemeny = DEF;                        //Esemény lekezelve, lenullázás
            delete[] K.regi;
            K.regi=K.uj;
            K.regi_size=K.uj_size;
            int x = 0;
            while (J.akt[x] != d)                 //Ellépegetünk J-ben a kicserélendõ élig
              {x=x+1;}
            J.akt[x]=c;                           //A nyitott élek vektorában kicseréljük a záródót a nyílóra

            if (robot1.grid_elek[c].dir == YMINUS || robot1.grid_elek[d].dir == YMINUS )
            {                                     //Ha a két él felett van az akadály

              K.temp = new double[K.uj_size];
              std::copy(K.uj, K.uj+x-1, K.temp);
              K.temp[x-1] = line+0.5;
              K.temp[x] = line+0.5;
              std::copy(K.uj+x+1, K.uj+K.uj_size , K.temp+x+1);
              K.uj = K.temp;
              K.temp = nullptr;

              if (memcmp(K.regi, K.uj, 8*K.uj_size) != 0 )
              {
                //ÚJ CELLA KÖVETKEZIK: KÉSZ
                tcella_tomb[cell_ix]=makecell(robot1.grid_elek[J.akt[x-1]],robot1.grid_elek[d],K.regi[x],line, cell_ix);
              }
            }

            if (robot1.grid_elek[c].dir == YPLUS || robot1.grid_elek[d].dir == YPLUS )
            {                                     
              K.temp = new double[K.uj_size];     //Ha a két él alatt van az akadály
              std::copy(K.uj, K.uj+x, K.temp);
              K.temp[x] = line+0.5;
              K.temp[x+1] = line+0.5;
              std::copy(K.uj+x+2, K.uj+K.uj_size , K.temp+x+2);
              K.uj = K.temp;
              K.temp = nullptr;
 
              if (memcmp(K.regi, K.uj, 8*K.uj_size) != 0 )
              {   
                //ÚJ CELLA KÖVETKEZIK: KÉSZ 
                tcella_tomb[cell_ix]=makecell(robot1.grid_elek[d],robot1.grid_elek[J.akt[x+1]],K.regi[x],line, cell_ix);
              }
            }

            // std::cout << "FOLYTATAS/1 J: \n";
            // for (int ii=0; ii < J.size; ii++)  std::cout << J.akt[ii] << "; ";
            // for (int ii=0; ii < K.uj_size; ii++)  std::cout << K.uj[ii] << "; ";
            // std::cout << " \n";
          }

          if (robot1.grid_elek[c].csucs2==csucsok[index])     //Ha van a vonalon csúcs, amibõl ZÁRUL ÉL
          {
            for (d = c+1; d < elszam; d=d+1)      //Megkeressük a csúcshoz tartozó másik élt
            {

              if (robot1.grid_elek[c].csucs2==robot1.grid_elek[d].csucs1) //Ha a másik él egy induló él
              {
                esemeny = FOLY2;
                break;
              }
              if (robot1.grid_elek[c].csucs2==robot1.grid_elek[d].csucs2) //Ha a másik él is záródó él
              {
                esemeny = ZAR;
                break;
              }
            }
          }

          if (esemeny==ZAR)                       //Ha a csúcsban két él ZÁRUL
          {
            esemeny = DEF;
            m_c = robot1.grid_elek[c].mered;      //c. él meredeksége
            m_d = robot1.grid_elek[d].mered;      //d. él meredeksége
            int cc=c, dd=d;
            if (m_c < m_d)                        //Új élek beillesztésének sorrendje a két meredekségtõl függ
            {
              cc = d;
              dd = c;
            }

            delete[] K.regi;
            K.regi=K.uj;
            K.regi_size=K.uj_size;
            int x = 0;

            while (J.akt[x] != cc)   x++;         //A két "élsorszám" kivétele a nyitott élek tömbjébõl
            
            if (J.size>2)
            {
              J.temp = new int[J.size-2];
              std::copy(J.akt, J.akt+x, J.temp); 
              std::copy(J.akt+x+2, J.akt+J.size, J.temp+x); 

              delete[] J.akt; 
              J.akt = J.temp;
              J.size = J.size-2;
              J.temp = nullptr;
            }
            else 
            {
              J.temp = nullptr;
              delete[] J.akt;
              J.akt = nullptr;
              J.size = 0;
            } 

            //Ha a két él között zárul cella
            if (robot1.grid_elek[c].dir >= XMINUS && robot1.grid_elek[d].dir == YMINUS )
            {
              if (K.regi_size <=2 )
              {
                delete[] K.regi;
                K.uj = nullptr;
                K.regi = nullptr;
                K.uj_size = 0;
                K.regi_size = 0;
              }
              else 
              {
                K.temp=new double[K.uj_size -2];  //Cellakezdetek frissítése               
                std::copy(K.uj, K.uj+x, K.temp); 
                std::copy(K.uj+x+2, K.uj+K.uj_size, K.temp+x); 
                K.uj = K.temp;
                K.temp = nullptr;
                K.uj_size = K.uj_size-2;
              }

              if (robot1.grid_elek[c].dir==YPLUS) //Ha nem csak a cellakezdetek frissítése kell
              {
              //ÚJ CELLA KÖVETKEZIK: KÉSZ 
              tcella_tomb[cell_ix]=makecell(robot1.grid_elek[c],robot1.grid_elek[d],K.regi[x],line,cell_ix);
              }
            }

            if (robot1.grid_elek[d].dir == YPLUS) //Záruló élek fölé kell cella
            {
              K.temp = new double[K.uj_size -2];  //Cellakezdetek frissítése
              std::copy(K.uj, K.uj+x-1, K.temp);
              K.temp[x-1] = line +0.5;
              K.temp[x] = line+0.5;
              std::copy(K.uj+x+3, K.uj+K.uj_size, K.temp+x+1); 
              K.uj = K.temp;
              K.temp = nullptr;
              K.uj_size = K.uj_size-2;

              //ÚJ CELLA KÖVETKEZIK: KÉSZ 
              tcella_tomb[cell_ix]=makecell(robot1.grid_elek[d],robot1.grid_elek[J.akt[x]],K.regi[x+1],line,cell_ix);

              if (robot1.grid_elek[c].dir == YMINUS && K.regi[x] <= line +0.5)
              {
                //ÚJ CELLA KÖVETKEZIK: KÉSZ 
                tcella_tomb[cell_ix]=makecell(robot1.grid_elek[J.akt[x-1]],robot1.grid_elek[c],K.regi[x-1],line,cell_ix);
              }
            }
            // std::cout << "ZARAS J: \n";
            // for (int ii=0; ii < J.size; ii++)  std::cout << J.akt[ii] << "; ";
            // for (int ii=0; ii < K.uj_size; ii++)  std::cout << K.uj[ii] << "; ";
            // std::cout << " \n";
          }

          if (esemeny==FOLY2)                     //Ha a csúcsban egy él zárul, egy él nyílik
          {
            esemeny = DEF;                        //Esemény lekezelve, lenullázás
            delete[] K.regi; 
            K.regi=K.uj;
            K.regi_size=K.uj_size;
            int x = 0;

            while (J.akt[x] != c)                 //Ellépegetünk J-ben a kicserélendõ élig
              {x=x+1;}
            J.akt[x]=d;                           //A nyitott élek vektorában kicseréljük a záródót a nyílóra

            if (robot1.grid_elek[c].dir == YMINUS || robot1.grid_elek[d].dir == YMINUS )
            {                                     //Ha a két él felett van az akadály

              K.temp = new double[K.uj_size];             
              std::copy(K.uj, K.uj+x-1, K.temp);
              K.temp[x-1] = line+0.5;
              K.temp[x] = line+0.5;
              std::copy(K.uj+x+1, K.uj+K.uj_size , K.temp+x+1);
              K.uj = K.temp;
              K.temp = nullptr;
              
              if (memcmp(K.regi, K.uj, 8*K.uj_size) != 0 )
              {
                //ÚJ CELLA KÖVETKEZIK: KÉSZ
                tcella_tomb[cell_ix]=makecell(robot1.grid_elek[J.akt[x-1]],robot1.grid_elek[c],K.regi[x],line,cell_ix);
              }
            }

            if (robot1.grid_elek[c].dir == YPLUS || robot1.grid_elek[d].dir == YPLUS )
            {                                     //Ha a két él alatt van az akadály
              
              K.temp = new double[K.uj_size];
              std::copy(K.uj, K.uj+x, K.temp);
              K.temp[x] = line+0.5;
              K.temp[x+1] = line+0.5;
              std::copy(K.uj+x+2, K.uj+K.uj_size , K.temp+x+2);
              K.uj = K.temp;
              K.temp = nullptr;
              
                if (memcmp(K.regi, K.uj, 8*K.uj_size) != 0 )
              {
                //ÚJ CELLA KÖVETKEZIK!  
                tcella_tomb[cell_ix]=makecell(robot1.grid_elek[c],robot1.grid_elek[J.akt[x+1]],K.regi[x],line,cell_ix);
              }
            }

            // std::cout << "FOLYTATAS/2 K: \n";
            // for (int ii=0; ii < J.size; ii++)  std::cout << J.akt[ii] << "; ";
            // for (int ii=0; ii < K.uj_size; ii++)  std::cout << K.uj[ii] << "; ";
            // std::cout << " \n";
          }     
        }
      }

      robot1.trapezok = new t_cella[cell_ix];       //Megfelelő méretű tömb                 
               
      std::copy(tcella_tomb, tcella_tomb+cell_ix, robot1.trapezok);     //Trapéz adatok másolása
      delete[] tcella_tomb;
      delete[] csucsok;  
      ROS_INFO("Trapez cellak sikeresen felveve! Osszesen %d darab trapez cella keletkezett", cell_ix);

      /*for (int ii=0; ii < cell_ix; ii++)          //TRAPÉZ cellák kiírása
      {
        ROS_INFO("%d. trapez cella: x1=%f, y1 = %f, m1 = %f, x2=%f, y2=%f, m2=%f", ii+1, robot1.trapezok[ii].cs_bal.x,
        robot1.trapezok[ii].cs_bal.y,robot1.trapezok[ii].m_bal, robot1.trapezok[ii].cs_jobb.x, 
        robot1.trapezok[ii].cs_jobb.y, robot1.trapezok[ii].m_jobb );
      } */
    }

//Trapéz cellákból Boustrophedon cellák megalkotása-------------------------------------------
  if (robot1.proc_statusz == BOUSTROPHEDON )
    {
      robot1.proc_statusz = INIT_POZ_FOGAD;
      ROS_INFO("Kovetkezzenek a Boustrophedon cellak!");
      //Boustrophedon cellák + cellaszélek meghatározása

      //robot1.trapezok : trapéz cellák adatai
      //cell_ix :         trapéz cellák száma
      //robot1.atmero :   robot átmérője

      bool erintett[cell_ix] = {0};                         //Trapéz cellák felhasználását jelzõ vektor
      poli *B_cellak = new poli[cell_ix];                   //Boustrophedon cellákat tartalmazó tömb
      t_cella *B_szelek = new t_cella[cell_ix];             //Boustro. cellaszéleket tartalmazó tömb

      for (int i = 0; i< cell_ix; i++)                      //Végigmegyünk az összes trapéz cellán
      {
        if (erintett[i]==0)                                 //Csak azzal foglalkozunk, ami még nem érintett
        {
          erintett[i] = 1;                                  //Trapéz cella érintésének jelzése
          int elozo = i;

          csucstomb poli_cs_1;                              //Alsó csúcsok tömbje
          csucstomb poli_cs_2;                              //Felső csúcsok tömbje
          poli_cs_1.akt = new geometry_msgs::Point32[2];
          poli_cs_2.akt = new geometry_msgs::Point32[2];

          poli_cs_1.size = 2;
          poli_cs_2.size = 2;

          poli_cs_1.akt[0] = robot1.trapezok[i].cs_bal;     //Alsó, felsõ csúcsok koordinátái
          poli_cs_1.akt[1] = robot1.trapezok[i].cs_jobb;

          poli_cs_2.akt[0].x = robot1.trapezok[i].cs_jobb.x;
          poli_cs_2.akt[0].y = robot1.trapezok[i].cs_jobb.y + robot1.trapezok[i].m_jobb;
          poli_cs_2.akt[1].x = robot1.trapezok[i].cs_bal.x;
          poli_cs_2.akt[1].y = robot1.trapezok[i].cs_bal.y + robot1.trapezok[i].m_bal;


          for (int j = i+1; j< cell_ix; j++)                //Passzoló trapéz cellák keresése
          {
            if (robot1.trapezok[elozo].cs_jobb.x == robot1.trapezok[j].cs_bal.x) 
            {                                               //Ha egyvonalban van a következõvel
              
              if ((robot1.trapezok[elozo].cs_jobb == robot1.trapezok[j].cs_bal) &&
                  (robot1.trapezok[elozo].m_jobb == robot1.trapezok[j].m_bal))
              {                                             //Közös oldalak egybevágóak
                elozo = j;
                erintett[j] = 1;

                //Csúcshalmazok frissítése: alsó csúcsok
                poli_cs_1.temp = new geometry_msgs::Point32[poli_cs_1.size+1];
                std::copy(poli_cs_1.akt, poli_cs_1.akt+poli_cs_1.size, poli_cs_1.temp);
                poli_cs_1.size++;
                poli_cs_1.temp[poli_cs_1.size] = robot1.trapezok[j].cs_jobb;

                delete[] poli_cs_1.akt; 
                poli_cs_1.akt = poli_cs_1.temp;
                poli_cs_1.temp = nullptr;

                //Csúcshalmazok frissítése: felső csúcsok
                poli_cs_2.temp = new geometry_msgs::Point32[poli_cs_2.size+1];
                poli_cs_2.temp[0].x = robot1.trapezok[j].cs_jobb.x;
                poli_cs_2.temp[0].y = robot1.trapezok[j].cs_jobb.y + robot1.trapezok[j].m_jobb;
                std::copy(poli_cs_2.akt, poli_cs_2.akt+poli_cs_2.size, poli_cs_2.temp+1);
                poli_cs_2.size++;
  
                delete[] poli_cs_2.akt; 
                poli_cs_2.akt = poli_cs_2.temp;
                poli_cs_2.temp = nullptr;
              }
              //Ha szomszédosak, de nem illeszkedik pont a következõ cella:
              else if ((robot1.trapezok[elozo].cs_jobb.y <= robot1.trapezok[j].cs_bal.y+robot1.trapezok[j].m_bal) &&
                      ( robot1.trapezok[elozo].cs_jobb.y+robot1.trapezok[elozo].m_jobb >= robot1.trapezok[j].cs_bal.y))
              {         
                int szamlalo1=0;                            //Aktuális cellának hány szomszédja van jobbról
                int szamlalo2=0;                            //Jobb oldali cellának hány szomszédja balról

                for (int ii=0; ii<cell_ix; ii++)
                {
                  if ((robot1.trapezok[elozo].cs_jobb.x == robot1.trapezok[ii].cs_bal.x) && 
                      (robot1.trapezok[elozo].cs_jobb.y <= robot1.trapezok[ii].cs_bal.y+robot1.trapezok[ii].m_bal) &&
                      (robot1.trapezok[elozo].cs_jobb.y+robot1.trapezok[elozo].m_jobb >= robot1.trapezok[ii].cs_bal.y))
                  {
                    szamlalo1++;                            //Aktuális cellának +1 jobb oldali szomszéd
                  }

                  if ((robot1.trapezok[ii].cs_jobb.x == robot1.trapezok[j].cs_bal.x) && 
                      (robot1.trapezok[ii].cs_jobb.y <= robot1.trapezok[j].cs_bal.y+robot1.trapezok[j].m_bal) &&
                      (robot1.trapezok[ii].cs_jobb.y+robot1.trapezok[ii].m_jobb >= robot1.trapezok[j].cs_bal.y))
                  {
                    szamlalo2++;                            //Aktuális cellának +1 bal oldali szomszéd
                  }
                }

                if (szamlalo1 == 1 && szamlalo2 == 1)       //Ha csak egymás szomszédjai
                {
                  elozo = j;
                  erintett[j] = 1;                          //Cella érintve
                  //Koordináták frissítése

                  //Csúcshalmazok frissítése: alsó csúcsok
                  poli_cs_1.temp = new geometry_msgs::Point32[poli_cs_1.size+2];
                  std::copy(poli_cs_1.akt, poli_cs_1.akt+poli_cs_1.size, poli_cs_1.temp);

                  poli_cs_1.size = poli_cs_1.size+2;
                  poli_cs_1.temp[poli_cs_1.size-2] = robot1.trapezok[j].cs_bal;
                  poli_cs_1.temp[poli_cs_1.size-1] = robot1.trapezok[j].cs_jobb;
                  
                  delete[] poli_cs_1.akt; 
                  poli_cs_1.akt = poli_cs_1.temp;
                  poli_cs_1.temp = nullptr;


                  //Csúcshalmazok frissítése: felső csúcsok
                  poli_cs_2.temp = new geometry_msgs::Point32[poli_cs_2.size+2];
                  poli_cs_2.temp[0].x = robot1.trapezok[j].cs_jobb.x;
                  poli_cs_2.temp[0].y = robot1.trapezok[j].cs_jobb.y + robot1.trapezok[j].m_jobb;
                  poli_cs_2.temp[1].x = robot1.trapezok[j].cs_bal.x;
                  poli_cs_2.temp[1].y = robot1.trapezok[j].cs_bal.y + robot1.trapezok[j].m_bal;

                  std::copy(poli_cs_2.akt, poli_cs_2.akt+poli_cs_2.size, poli_cs_2.temp+2);
                  poli_cs_2.size = poli_cs_2.size+2;

                  delete[] poli_cs_2.akt;
                  poli_cs_2.akt = poli_cs_2.temp;
                  poli_cs_2.temp = nullptr;
                }
              }
            }
          }

          //Koordináták összesítése
          B_cellak[b_szam].db = poli_cs_1.size + poli_cs_2.size;
          B_cellak[b_szam].cs = new geometry_msgs::Point32[B_cellak[b_szam].db];

          //Boustrophedon cella csúcsainak kigyűjtése
          std::copy(poli_cs_1.akt, poli_cs_1.akt+poli_cs_1.size, B_cellak[b_szam].cs);
          std::copy(poli_cs_2.akt, poli_cs_2.akt+poli_cs_2.size, B_cellak[b_szam].cs+poli_cs_1.size);

          b_szam = b_szam +1;

          delete[] poli_cs_1.akt; 
          delete[] poli_cs_2.akt;  
          poli_cs_1.akt = nullptr;
          poli_cs_2.akt = nullptr;

          //Virtuális cella: nem foglalkozok vele, nem fordul elő.
          //Régiók sem keletkeznek a méretcsökkentéssel, nem foglalkozom vele.
        }
      }

      robot1.bcellak_nagy = new poli[b_szam];               //Foglalás a szűrt csúcsú NAGY poligon celláknak
      robot1.bcellak_k = new poli[b_szam];                  //Foglalás a méretcsökkentett poligonoknak

      for (int i = 0; i < b_szam; i++)                      //Felesleges csúcsok kiszedése + méretcsökkentés
      {
        robot1.bcellak_nagy[i].db= B_cellak[i].db;

        //Felesleges (ismétődő/ egyenesre eső) csúcsok kiszedése
        for (int j = 1; j < B_cellak[i].db-1; j++)        //Az első és az utolsó csúcs nem fog ismétlődni
        {
          if ((B_cellak[i].cs[j]==B_cellak[i].cs[j-1]) || //Előzővel megegyező csúcs vagy egy egyenesre esik
              (B_cellak[i].cs[j].y-B_cellak[i].cs[j-1].y == B_cellak[i].cs[j+1].y-B_cellak[i].cs[j].y))
            {robot1.bcellak_nagy[i].db--;} 
        }

        // Új foglalás csak a szükséges csúcsoknak- - - - - - - - - - - - - - - - - - - - - - - - -  - - -
        {
        robot1.bcellak_nagy[i].cs = new geometry_msgs::Point32[robot1.bcellak_nagy[i].db];
               
        robot1.bcellak_nagy[i].cs[0] = B_cellak[i].cs[0]; //Első csúcs tuti másolható át
        robot1.bcellak_nagy[i].cs[robot1.bcellak_nagy[i].db-1] = B_cellak[i].cs[B_cellak[i].db-1];//Utolsó is
        int j = 1;
        for (int k = 1; k< B_cellak[i].db-1; k++)         //Köztes csúcsok átmásolása
        {
          if ((B_cellak[i].cs[k]==B_cellak[i].cs[k-1]) || //Előzővel megegyező csúcs vagy egy egyenesre esik
              (B_cellak[i].cs[k].y-B_cellak[i].cs[k-1].y == B_cellak[i].cs[k+1].y-B_cellak[i].cs[k].y))
            { continue;  }                                //Nem szükséges csúcsok kihagyása a másolásból
          else 
          {
            robot1.bcellak_nagy[i].cs[j] = B_cellak[i].cs[k];  //Csúcs átmásolása 
            j++;
          }
        }
        delete[] B_cellak[i].cs;                          //Szűretlen csúcsok tömbjének törlése
        }

        //Cellák méretcsökkentése egyből -----------------EZ CSAK A GRIDES megoldáshoz megfelelő ! !         
        {
        robot1.bcellak_k[i].db = robot1.bcellak_nagy[i].db;
        robot1.bcellak_k[i].cs = new geometry_msgs::Point32[robot1.bcellak_k[i].db];               

        //Első csúcs mindig jobbra és fel tolódik.
        robot1.bcellak_k[i].cs[0].x = robot1.bcellak_nagy[i].cs[0].x + robot1.atmero/2;
        robot1.bcellak_k[i].cs[0].y = robot1.bcellak_nagy[i].cs[0].y + robot1.atmero/2;

        //Utolsó csúcs mindig jobbra és le tolódik.
        robot1.bcellak_k[i].cs[robot1.bcellak_k[i].db-1].x=robot1.bcellak_nagy[i].cs[robot1.bcellak_k[i].db-1].x+robot1.atmero/2;
        robot1.bcellak_k[i].cs[robot1.bcellak_k[i].db-1].y=robot1.bcellak_nagy[i].cs[robot1.bcellak_k[i].db-1].y-robot1.atmero/2;
      
        for (int j=1; j<robot1.bcellak_k[i].db-1;j++)     //Végig a maradék belső csúcsokon
        {
          if (robot1.bcellak_nagy[i].cs[j+1].x > robot1.bcellak_nagy[i].cs[j-1].x) //+y szituáció
          {
            if ((robot1.bcellak_nagy[i].cs[j+1].y > robot1.bcellak_nagy[i].cs[j-1].y))  //-x,+y
            {
              robot1.bcellak_k[i].cs[j].x = robot1.bcellak_nagy[i].cs[j].x - robot1.atmero/2;
              robot1.bcellak_k[i].cs[j].y = robot1.bcellak_nagy[i].cs[j].y + robot1.atmero/2;
            }
            else                                                                        //+x,+y
            {
              robot1.bcellak_k[i].cs[j].x = robot1.bcellak_nagy[i].cs[j].x + robot1.atmero/2;
              robot1.bcellak_k[i].cs[j].y = robot1.bcellak_nagy[i].cs[j].y + robot1.atmero/2;
            }
          }
          else                                                                      //-y szituáció
          {
            if ((robot1.bcellak_nagy[i].cs[j+1].y > robot1.bcellak_nagy[i].cs[j-1].y))  //-x,-y
            {
              robot1.bcellak_k[i].cs[j].x = robot1.bcellak_nagy[i].cs[j].x - robot1.atmero/2;
              robot1.bcellak_k[i].cs[j].y = robot1.bcellak_nagy[i].cs[j].y - robot1.atmero/2;
            }
            else                                                                        //+x,-y
            {
              robot1.bcellak_k[i].cs[j].x = robot1.bcellak_nagy[i].cs[j].x + robot1.atmero/2;
              robot1.bcellak_k[i].cs[j].y = robot1.bcellak_nagy[i].cs[j].y - robot1.atmero/2;
            }
          }
        }
        }

        //Cellaszélek meghatározása
        {
          B_szelek[i].cs_bal = robot1.bcellak_k[i].cs[0];      //Bal cellaszél adatok egyértelműek
          B_szelek[i].m_bal = robot1.bcellak_k[i].cs[robot1.bcellak_k[i].db-1].y - B_szelek[i].cs_bal.y;
          B_szelek[i].cs_jobb = B_szelek[i].cs_bal;                          
          double seged = -100; 

          for (int j=1; j< robot1.bcellak_k[i].db; j++)                   //Jobb szél adatok meghatározása
          {
            if (robot1.bcellak_k[i].cs[j].x == B_szelek[i].cs_jobb.x)     //Max x-hez tartozó y koordináták
            {
              if (robot1.bcellak_k[i].cs[j].y < B_szelek[i].cs_jobb.y)
              {
                B_szelek[i].cs_jobb.y = robot1.bcellak_k[i].cs[j].y;      //Max x-hez tartozó min y
              }
              if (seged < robot1.bcellak_k[i].cs[j].y )
              {
                seged = robot1.bcellak_k[i].cs[j].y;                      //Max x-hez tartozó max y 
              }
              B_szelek[i].m_jobb = seged - B_szelek[i].cs_jobb.y;         //Jobb szél magasság megállapítása
            }
            if (robot1.bcellak_k[i].cs[j].x > B_szelek[i].cs_jobb.x)      //Maximális x koodináta megkeresése
            {
              B_szelek[i].cs_jobb = robot1.bcellak_k[i].cs[j];
              seged = B_szelek[i].cs_jobb.y;
            }
          }       
        }

      }



      poli *B_cellak_ok = new poli[b_szam];

      //Végig a cella csúcsain: ismétlődő csúcsok kivétele
      for (int i = 0; i< b_szam; i++)
      {
        B_cellak_ok[i].db= robot1.bcellak_k[i].db;

        //Felesleges (ismétődő/ egyenesre eső) csúcsok kiszedése
        for (int j = 1; j < robot1.bcellak_k[i].db; j++)        //Az első és az utolsó csúcs nem fog ismétlődni
        {
          if (robot1.bcellak_k[i].cs[j]==robot1.bcellak_k[i].cs[j-1]) //Előzővel megegyező csúcs vagy egy egyenesre esik
            {B_cellak_ok[i].db--;} 
        }

        // Új foglalás csak a szükséges csúcsoknak- - - - - - - - - - - - - - - - - - - - - - - - -  - - -
        B_cellak_ok[i].cs = new geometry_msgs::Point32[B_cellak_ok[i].db];
               
        B_cellak_ok[i].cs[0] = robot1.bcellak_k[i].cs[0]; //Első csúcs tuti másolható át
        B_cellak_ok[i].cs[B_cellak_ok[i].db-1] = robot1.bcellak_k[i].cs[robot1.bcellak_k[i].db-1];//Utolsó is
        int j = 1;
        for (int k = 1; k< robot1.bcellak_k[i].db; k++)         //Köztes csúcsok átmásolása
        {
          if (robot1.bcellak_k[i].cs[k]==robot1.bcellak_k[i].cs[k-1])//Előzővel megegyező csúcs vagy egy egyenesre esik
            { continue;  }                                //Nem szükséges csúcsok kihagyása a másolásból
          else 
          {
            B_cellak_ok[i].cs[j] = robot1.bcellak_k[i].cs[k];  //Csúcs átmásolása 
            j++;
          }
        }
        delete[] robot1.bcellak_k[i].cs;                       //Szűretlen csúcsok tömbjének törlése
        robot1.bcellak_k[i].cs = B_cellak_ok[i].cs;
        robot1.bcellak_k[i].db = B_cellak_ok[i].db;
      }
           

      /*for (int ii = 0; ii< b_szam; ii++)
      {
        ROS_INFO("%d. cella szelei: x1: %lf, y1: %lf, m1 %lf, x2: %lf, y2: %lf, m2 %lf",ii+1, B_szelek[ii].cs_bal.x, B_szelek[ii].cs_bal.y,
        B_szelek[ii].m_bal, B_szelek[ii].cs_jobb.x, B_szelek[ii].cs_jobb.y, B_szelek[ii].m_jobb );
      }*/

      robot1.bszelek = B_szelek;
      B_szelek = nullptr;
      delete[] B_cellak;
      delete[] robot1.bcellak_k;
      robot1.bcellak_k = B_cellak_ok;
      B_cellak = nullptr;

      ROS_INFO("Boustrophedon cellak keszenallnak.");
      ROS_INFO("Most pedig tortenjen meg a pozicio inicializalas!");
    }

//Trapéz cellákból Boustrophedon cellák megalkotása-------------------------------------------
  if (robot1.proc_statusz == STARTCELLA )
    {
      robot1.proc_statusz = GRAFKERESO;

      //Melyik trapéz cellán belülre esik a pont:

      t_cella sc;                                     //A trapéz cella, amibe esik a pont

      for (int i = 0; i< cell_ix; i++)
      {                                               //A pont a trapézon belülre esik-e
        if (robot1.startpozicio.x >= robot1.trapezok[i].cs_bal.x && 
            robot1.startpozicio.x <= robot1.trapezok[i].cs_jobb.x &&
            robot1.startpozicio.y >= robot1.trapezok[i].cs_bal.y &&
            robot1.startpozicio.y <= robot1.trapezok[i].cs_bal.y + robot1.trapezok[i].m_bal)
            {
              startcella = i;
              //ROS_INFO("Trapez startcella sorszama: %d", i);
              sc = robot1.trapezok[i];                //Trapéz cella elmentése
              break;
            }
      }

      if (startcella == -1)   //HA egy cellát sem találtunk, azaz nem sikerult jol becsulni a poziciot.
      {
        ROS_INFO("A kezdopozicio egy akadaly teruletere lett megadva. Probaljuk meg ujra megadni.");
        robot1.proc_statusz = INIT_POZ_FOGAD;
      }

      //Az adott trapéz cella melyik boustrophedon cellának építőeleme:

      for (int i= 0; i< b_szam; i++)            //Végig a (nagy) BOustrophedon cellákon
      {
        bool also_match   =0;
        bool felso_match  =0;

        for (int j = 0; j < robot1.bcellak_nagy[i].db; j++)   //Végig a cellák csúcsain
        {
          //Ha a trapéz cella egyik alsó csúcsa szerepel a poligon cella csúcsai között
          if (sc.cs_bal == robot1.bcellak_nagy[i].cs[j]|| sc.cs_jobb == robot1.bcellak_nagy[i].cs[j])
            also_match = 1;
          
          //Ha a trapéz cella egyik felső csúcsa szerepel a poligon cella csúcsai között
          if ((sc.cs_bal.x==robot1.bcellak_nagy[i].cs[j].x) && (sc.cs_bal.y + sc.m_bal==robot1.bcellak_nagy[i].cs[j].y) || 
            (sc.cs_jobb.x==robot1.bcellak_nagy[i].cs[j].x) && (sc.cs_jobb.y + sc.m_jobb==robot1.bcellak_nagy[i].cs[j].y))
            felso_match = 1;
        }

        if (also_match && felso_match)      //Alul is felül is szerepel a trapéz cella vmelyik csúcsa 
        {
          ROS_INFO("Megvan a startcella! Sorszama nem mas, mint %d", i);
          startcella = i;                   //Startcella index beállítása
          break;                            //Csak egy cellában lehet, nem kell tovább vizsgálódni.
        }
      }  

      double dist, mindist;
      int minidx=0;
      //Mozgás az adott cella legközelebbi csúcsába!
      for (int j = 0; j< robot1.bcellak_k[startcella].db; j++)  //Legközelebbi csúcs megkeresése.
      {
        dist = sqrt(pow(robot1.bcellak_k[startcella].cs[j].x - robot1.startpozicio.x,2) + 
                    pow(robot1.bcellak_k[startcella].cs[j].y - robot1.startpozicio.y,2));
        if (j==0) mindist = dist;
        if (dist < mindist)
        {
          mindist = dist;
          minidx = j;
        }
      }

      //Újabb célpozíció hozzáillesztés, majd értékadás.
      robot1.ut.push_back(geometry_msgs::Pose());
      robot1.ut[robot1.ut.size()-1].position.x = robot1.bcellak_k[startcella].cs[minidx].x;
      robot1.ut[robot1.ut.size()-1].position.y = robot1.bcellak_k[startcella].cs[minidx].y;
    }

//Gráfkereső algoritmus: cellákból álló gráf megalkotása, majd DFS----------------------------
  if (robot1.proc_statusz == GRAFKERESO )
    {
      robot1.proc_statusz = UTKERESO;

      ROS_INFO("Kovetkezzen a grafkereses!");
      //robot1.bszelek                            //Boustrophedon cellák széleinek mátrixa
      //b_szam                                    //Boustrophedon cellák száma

      Graf g(b_szam,b_szam);                      //Üres szomszédossági mátrix létrehozása
      std::vector<bool> elert_csucsok(b_szam, 0); //Elért csúcsokat jelző vektor

      for (int i = 0; i< b_szam-1;i++)            //Cellák vizsgálata kettesével
      {
        for (int j = i+1; j < b_szam;j++)
        {
          //Ha szomszédosak: szomszédos x koordináták és azonos magasságban vannak
          if ((robot1.bszelek[i].cs_jobb.x + robot1.atmero == robot1.bszelek[j].cs_bal.x) &&
          (robot1.bszelek[i].cs_jobb.y <=robot1.bszelek[j].cs_bal.y+robot1.bszelek[j].m_bal) &&
          (robot1.bszelek[i].cs_jobb.y+robot1.bszelek[i].m_jobb >= robot1.bszelek[j].cs_bal.y))
          {
            g.uj_el(i,j);
          }
          //Virtuális cellák: jelenleg nincsenek, implementáció halasztása.
        }
      }
      ROS_INFO("Graf matrix az elek adataival feltoltve.");

      int elerheto = 0;
      int bejarando[b_szam];
      for (int i=0; i< b_szam; i++)  bejarando[i]=-1;

      elerheto = g.DFS(startcella,elert_csucsok, bejarando, 0); //DFS keresés

      ROS_INFO("Ennyi csucsot kell bejarni: %d. Maguk a csucsok:", elerheto);
      for (int i = 0; i< elerheto; i++) std::cout << bejarando[i] << "; ";
      std::cout<< "\n";

      int i = 0, s_end = 0;
      int Sorrend[2*b_szam];
      Sorrend[0] = bejarando[0];                      //Kezdõcella megegyezik
      for (int j=1;j<2*b_szam;j++) Sorrend[j] = -1;   //Legyen egyertelmu, meddig tart           

      while (Sorrend[s_end] != bejarando[elerheto-1]) //Amíg nem ugyanaz a kettõ vége
      { 

        bool temp_felt=0;
        for (int k=0 ;k<s_end; k++)                   //Már bejárt csúcsok átnézése
        {
          if (Sorrend[s_end]==bejarando[i+1]) temp_felt =1;
        }

        if ( i+1 != elerheto-1 && temp_felt)
          {
            //Ha már be van járva a cella vagy már szerepel az útban és nem az utolsó
          }
        else if (g.szomszedos(Sorrend[s_end],bejarando[i+1]) ==0) //Ha nem szomszédos az elõzõ és a vizsgált
        {
          //Legrövidebb út az aktuális cellából a következõbe
          int atmenet[b_szam];                          //Tömb az átmeneti celláknak
          for (int l=0;l<b_szam;l++) atmenet[l] = -1;   //Értékek hibakezeléshez

          int db = g.ShPath(Sorrend[s_end], bejarando[i+1],atmenet); //Legrövidebb út keresés
          for (int k =0; k < db; k++)               //Átmeneti cellák beíírása
          {                                         //(atmenet tombbe az elso elem nem kerul bele)
            s_end++;
            Sorrend[s_end] = atmenet[k];
          }
        }
        else
        {
          s_end++;
          Sorrend[s_end] = bejarando[i+1];        //Ha szomszédosak: sima hozzáadás  
        }
        i++;                                      //Index növelése
      }     

      ut_szam = s_end+1;
      robot1.utmx = new utmatrix[ut_szam];        //Útmátrix létrehozása

      ROS_INFO("Kiegeszitett sorrend ennyi elembol all: %d. Maguk a csucsok:", ut_szam);
      for (int i = 0; i<ut_szam; i++)
      {
        robot1.utmx[i].sorszam = Sorrend[i];
        robot1.utmx[i].elert = 0;
        std::cout << Sorrend[i] << "; ";
      }
      std::cout<< "\n";
      ROS_INFO("Kovetkezzen az utkereses: a cella be- es kilepesi pontjainak meghatarozasa.");
    }

//Útkereső algoritmus: cella be-és kilépési pontjainak meghatározása--------------------------
  if (robot1.proc_statusz == UTKERESO )
    {
      robot1.proc_statusz = STARTPONTBA;

      //Útmátrix létrehozva a gráfkereső végén
      //Sorszám adatok kitöltve a gráfkereső végén

      for (int i=0; i<ut_szam; i++)                               //Ismétlődő cellák megjelölése
      {
        int akt = robot1.utmx[i].sorszam;
        for (int j=i+1; j< ut_szam; j++)                        //Ismételten előforduló cellasorszámok keresése
        {
          if (akt == robot1.utmx[j].sorszam) robot1.utmx[j].elert = 1; //Imsétlődő cella jelzése

          //Megj: virtuális celláktól most nem kell tartani.
        }
      }

      if (ut_szam == 1)                                           //Ha csak egyetlen cella van:
      {
        robot1.utmx[0].be = robot1.bszelek[startcella].cs_bal;
        robot1.utmx[0].ki = robot1.bszelek[startcella].cs_jobb;
      }
      else                                                        //Egyébként: útmátrix feltöltése adatokkal
      {
        if (robot1.bszelek[robot1.utmx[1].sorszam].cs_bal.x > robot1.bszelek[startcella].cs_bal.x)
          robot1.utmx[0].be = robot1.bszelek[startcella].cs_bal;  //Elsõ cella belépési pont: BAL startpont
        else
          robot1.utmx[0].be = robot1.bszelek[startcella].cs_jobb; //Elsõ cella belépési pont: JOBB startpont
        
        for (int i=0; i<ut_szam-1; i++)                           //Ki- és belépési pontok meghatározása
        {

          bool felt=(robot1.bszelek[robot1.utmx[i].sorszam].cs_bal.x > robot1.bszelek[robot1.utmx[i+1].sorszam].cs_bal.x);
          switch (felt)
          {
            case 0:                                               //Ha jobbra lépünk át a következõ cellába
            {                                                     //Ki- és belépési pontok x koordinátái adódnak
              robot1.utmx[i].ki.x   = robot1.bszelek[robot1.utmx[i].sorszam].cs_jobb.x;
              robot1.utmx[i+1].be.x = robot1.bszelek[robot1.utmx[i+1].sorszam].cs_bal.x;

              //Ha az új cella alsó csúcsa magasabban van..
              if (robot1.bszelek[robot1.utmx[i+1].sorszam].cs_bal.y>robot1.bszelek[robot1.utmx[i].sorszam].cs_jobb.y)
              {                                                   
                //Legyen ez a belépési pont y-ja
                robot1.utmx[i].ki.y = robot1.bszelek[robot1.utmx[i+1].sorszam].cs_bal.y;
              }
              else
              {
                //Ha nem, akkor a régi cella alsó csúcsának y-ja legyen
                robot1.utmx[i].ki.y = robot1.bszelek[robot1.utmx[i].sorszam].cs_jobb.y;
              }
              //Az akt. cella kilépési pont y-ja egyezzen meg az elõzõvel 
              robot1.utmx[i+1].be.y = robot1.utmx[i].ki.y;
              break;
            }

            case 1:                                               //Ha balra lépünk át a következõ cellába 
            {                                                     //Ki- és belépési pontok x koordinátái adódnak
              robot1.utmx[i].ki.x   = robot1.bszelek[robot1.utmx[i].sorszam].cs_bal.x;
              robot1.utmx[i+1].be.x = robot1.bszelek[robot1.utmx[i+1].sorszam].cs_jobb.x;

              //Ha az új cella alsó csúcsa magasabban van..
              if (robot1.bszelek[robot1.utmx[i+1].sorszam].cs_jobb.y>robot1.bszelek[robot1.utmx[i].sorszam].cs_bal.y)
              {
                //Legyen ez a belépési pont y-ja
                robot1.utmx[i].ki.y = robot1.bszelek[robot1.utmx[i+1].sorszam].cs_jobb.y;
              }
              else
              {
                //Ha nem, akkor a régi cella alsó csúcsának y-ja legyen
                robot1.utmx[i].ki.y = robot1.bszelek[robot1.utmx[i].sorszam].cs_bal.y;
              }
              //Az akt. cella kilépési pont y-ja egyezzen meg az elõzõvel 
              robot1.utmx[i+1].be.y = robot1.utmx[i].ki.y;
              break;
            }
          }
          //Ha a vizsgált cella virtuális cella: implementálás nem szükséges még.
        }
        //Utolsó cella kilépési pontjának beállítása
        robot1.utmx[ut_szam-1].ki = robot1.bszelek[robot1.utmx[ut_szam-1].sorszam].cs_jobb;   
      }

      ROS_INFO("Elkeszult az utmatrix. Mehetunk a startpontba.");
      /*std::cout << "Sorszam\t" << "Ismetelt?\t" << "Belepesi pont\t" << "Kilepesi pont" << "\n";

      for (int i = 0; i< ut_szam; i++)
      {
        std::cout << robot1.utmx[i].sorszam << "\t   " << robot1.utmx[i].elert << "\t\t" << robot1.utmx[i].be.x << ";" <<
        robot1.utmx[i].be.y << "\t" << robot1.utmx[i].ki.x << ";" << robot1.utmx[i].ki.y << "\n";
      } */
    }

//Startpontba függvény:-----------------------------------------------------------------------
  if (robot1.proc_statusz == STARTPONTBA )
    {
      robot1.proc_statusz = UTSZAMOLAS;

      poli c = robot1.bcellak_k[robot1.utmx[0].sorszam];      //Aktuális cella adatai: poligon-csúcsok
      geometry_msgs::Point32 p;                               //Aktuális pozíció
      p.x = robot1.startpozicio.x; 
      p.y = robot1.startpozicio.y;                
      geometry_msgs::Point32 cel = robot1.utmx[0].be;         //Célpozíció

      //ELOKESZULETEK
      el p_elek[c.db];
      poli_elkereso(p_elek, c.db, c.cs);                      //Poligon éleinek megkeresése
      std::sort(p_elek, p_elek+c.db, comparePoliEdges);       //Élek rendezése x1,x2,y1 szerint


      // for (int ix=0;ix < c.db; ix++)                       
      // {   
      //   ROS_INFO("El #%d: cs1: %f;%f  cs2: %f,%f dir: %d m: %f",ix, p_elek[ix].csucs1.x, p_elek[ix].csucs1.y,
      //     p_elek[ix].csucs2.x, p_elek[ix].csucs2.y, p_elek[ix].dir, p_elek[ix].mered);
      // }

      int mode;
      if (p.x == cel.x)       mode=0;                         //Függőleges lépésre van csak szükség
      else if (cel.x > p.x)   mode=1;                         //Áthaladás a cella jobb szélére
      else                    mode=2;                         //Áthaladás a cella bal szélére

      //Függőleges lépés a startpontba
      if (mode==0)                                            //Lépés lefele, a startpontba.
      {
        //Újabb célpozíció hozzáillesztés, majd értékadás.
        robot1.addNode(cel);
      }

      else
      {

        bool x_alul_jel = 0;                                    //Jelzés, hogy a kezdőél megvan
        double x_max = c.cs[0].x;                               //Maximális x érték tárolása
        std::vector<int> x_alul = {};                           //Élek tárolása vektorral.

        geometry_msgs::Point32 x_max_also = robot1.bszelek[robot1.utmx[0].sorszam].cs_jobb;//Maximális x érték tárolása

        bool reverse = 0;                                       //Segédváltozó: jobbról balra haladáshoz
        bool elso = 1;
        for (int ii=0; ii<c.db; ii++)                           //Alsó élsorozatok megállapítása
        {
          //Meredekségeket nem kell már számolni.

          if (reverse) break;                                   //Készen vagyunk a vizsgálandó élekkel. 


          //Alsó  élsorozat kezdő elemének kiszámolása
          if (p_elek[ii].csucs1.x == p_elek[0].csucs1.x && elso && 
             (p_elek[ii].dir == YPLUS || (p_elek[ii].dir==XMINUS && p_elek[ii].csucs2.y > p_elek[ii].csucs1.y ))) 
          {
            x_alul.push_back(ii);  
            elso = 0;                             
          }
          
          //Élsorozat teljes meghatározása: ha ez az él következik
          if (p_elek[ii].csucs1 != x_max_also && x_alul.size() !=0 && 
              p_elek[ii].csucs1 == p_elek[x_alul.back()].csucs2 && p_elek[ii].dir != YMINUS)
          {
            switch (mode)
            {
            case 1:                                             //Ha a jobb alsó csúcs a startpont
              {
                x_alul.push_back(ii);                           //Alsó él indexének hozzáadása a végére

                if (p_elek[x_alul.back()].csucs1.x <= robot1.startpozicio.x &&
                    p_elek[x_alul.back()].csucs2.x >= robot1.startpozicio.x ) 
                  {
                    x_alul.clear();                             //Eddigi élek törlése
                    x_alul.push_back(ii);                       //Kezdőél megtalálása
                  }
                break;
              }
            case 2:                                             //Ha a bal alsó csúcs a startpont
              {
                if (p_elek[x_alul.front()].csucs2.x >= robot1.startpozicio.x)  
                {break;}                                        //HA az első él vonalában vagyunk

                x_alul.push_back(ii);                           //Alsó él indexének hozzáadása a végére

                if (p_elek[x_alul.back()].csucs1.x <= robot1.startpozicio.x &&
                    p_elek[x_alul.back()].csucs2.x >= robot1.startpozicio.x )
                  {
                    reverse = 1;                                //Ha elértünk a pozíció magasságába, kész
                  }
                
              }
            }
          }
        }
        if (reverse)                                            //Meg kell fordítani az indexek sorrendjét.
        {
          std::vector<int> seged = x_alul;
          for (int k = 0; k< x_alul.size(); k++)
          {
            x_alul[k] = seged[x_alul.size()-1-k];
          }
        }

        //STARTPONTBA HALADÁS

        //Aktuális pozícióból lefele haladás a szembekerülő élig, majd haladás a csúcsába
        if ( p_elek[x_alul.front()].mered == 100)               //Függőleges él fele tartunk
        {
          switch (mode)
          {
          case 1:
            robot1.addNode(p_elek[x_alul.front()].csucs2);      //Újabb célpozíció hozzáillesztés, majd értékadás.
            break;
          case 2:
            robot1.addNode(p_elek[x_alul.front()].csucs1);      //Újabb célpozíció hozzáillesztés, majd értékadás.
            break;
          }
        }
        else                                                    //Nem függőleges él kerül szembe
        {
          geometry_msgs::Point32 temp;
          temp.x = robot1.ut.back().position.x;
          temp.y = p_elek[x_alul.front()].csucs1.y + (temp.x-p_elek[x_alul.front()].csucs1.x)*p_elek[x_alul.front()].mered;
          robot1.addNode(temp);                                 //Újabb célpozíció hozzáillesztés, majd értékadás.

          switch (mode)
          {
          case 1:                                               //Haladás JOBBRA a szembekerülő él mentén a csúcsáig 
            robot1.addNode(p_elek[x_alul.front()].csucs2);      //Újabb célpozíció hozzáillesztés, majd értékadás.
            break;
          case 2:
            robot1.addNode(p_elek[x_alul.front()].csucs1);      //Újabb célpozíció hozzáillesztés, majd értékadás.
            break;
          }
        }

        if (x_alul.size() > 1)                                  //Ha még nem érkeztünk meg a startpontba
        {
          for (int i=1;i<x_alul.size(); i++)                    //Végighaladás az alsó éleken
          {
            switch (mode)
            {
            case 1:
              robot1.addNode(p_elek[x_alul[i]].csucs2);         //Új csúcsok hozzáadása az útvonalhoz.
              break;
            
            case 2:
              robot1.addNode(p_elek[x_alul[i]].csucs1);         //Új csúcsok hozzáadása az útvonalhoz.
              break;
            }
            
          }
        }
      }
      ROS_INFO("A robot utja startpozicioig eloallitva.");
    }

//Útszámolás függvény:------------------------------------------------------------------------
  if (robot1.proc_statusz == UTSZAMOLAS)
  {
    robot1.proc_statusz = ELINDULAS;

    int srend[ut_szam];                               //Sorrend kimásolása
    for (int i = 0; i<ut_szam; i++)
    {
      srend[i]= robot1.utmx[i].sorszam;               //cella sorszám kimásolása
    }

    for (int i = 0; i< ut_szam-1; i++)                //Végig az összes cellán
    {
      
      // STEP_A: a startpontból indulva a cella bejárása / áthaladás rajta

      if (robot1.utmx[i].elert == 0)                  //Még nincs bejárva a cella
      {
        robot1.cellatbejar(srend[i], i);
        robot1.utmx[i].elert = 1;                     
      } 
      else                                            //Már be van járva a cella
      { 
        robot1.athaladas(srend[i], i);                //Áthaladás függvény
      }

      bool vege = 1;
      for (int j = i+1; j < ut_szam; j++)           //Ha egy cella is hiányzik még, nincs vége
      {
        if (robot1.utmx[j].elert == 0) vege = 0;
      }

      if (vege) 
      {
        break;                                //Ha nincs bejáratlan cella, kiugrás a ciklusból
      }

      // STEP_B: mozgás a cella kilépési pontjába

      if (robot1.ut.back().position.x == robot1.utmx[i].ki.x) //Függőleges mozgás
      {
        robot1.addNode(robot1.utmx[i].ki);            //Függőleges mozgás: új csúcs útvonalhoz adása
      }
      else                                            //Nem függőleges mozgás, hanem át a cellán
      {                                 
        robot1.athaladas(srend[i], i);                //Áthaladás függvény
      }

      // STEP_C: átlépés a következő cella belépési pontjába
      robot1.addNode(robot1.utmx[i+1].be);            //Sima vízszintes, egységnyi lépés

      //STEP D: mozgás a bejárás startpontjába: cellátbejár függvényen belül!
    }

    if (robot1.utmx[ut_szam-1].elert == 0)            //Utolsó cella bejárása külön, ha még nem volt
    {
      robot1.cellatbejar(srend[ut_szam-1], ut_szam-1);  
    }
  }

//Startpozícióban a robot, első cél kiküldése-------------------------------------------------
  if (robot1.proc_statusz==ELINDULAS)
    {
      robot1.proc_statusz = TORLES;
      robot1.poz_eltolas();                             //ÚTVONAL BELJEBB HOZATALA
      aktpoz.pose.position = robot1.startpozicio;       //Pozíció frissítés.
      goal.header.frame_id="map";                       //Célpozíció beállítás
      goal.pose = robot1.ut[0];

      //Első cél kiírása
      ROS_INFO("CEl #%d: x:%lf y: %lf",1, goal.pose.position.x, goal.pose.position.y);  

      pub_goal.publish(goal);                           //Első cél kiadása
      robot1.celbaert = 0;                              //Elindulás jelzése
    }

//Területfedés előtt: dinamikusan foglalt memóriák törlése, program vége.---------------------
  if (robot1.proc_statusz == TORLES)
    {
      robot1.proc_statusz = BEJARAS;


      for (int ii = 0; ii< b_szam; ii++)
        {
          // ROS_INFO("%d. cella csucsai:", ii+1);
          // for (int jj = 0; jj < robot1.bcellak_k[ii].db; jj++)
          // {
          //   std::cout << "(" << robot1.bcellak_k[ii].cs[jj].x <<";" << robot1.bcellak_k[ii].cs[jj].y <<")  " ; 
          // }
          // std::cout << "\n";
          delete[] robot1.bcellak_nagy[ii].cs;          //Csúcsoknak foglalt memória felszabadítása
          delete[] robot1.bcellak_k[ii].cs;
        }

      delete[] robot1.grid_elek;                        //MEMÓRIA_FELSZABADÍTÁSOK
      delete[] robot1.trapezok;
      delete[] robot1.bcellak_nagy;                     
      delete[] robot1.bcellak_k; 
      delete[] robot1.bszelek;               
      delete[] robot1.utmx;
    }

//Aktuális célpozíció elérve, következő elérendő pozíció kiküldése egyszer--------------------
  if (robot1.proc_statusz == BEJARAS && robot1.celbaert==1 ) { 

      if (index == robot1.ut.size())                    //Ha az utolsó elérendő pontnál járunk
      {
        robot1.proc_statusz = URES;
        break;                                          //Kilépés a ciklusból: program vége
      }
      geometry_msgs::Point elozopoz=aktpoz.pose.position;
      aktpoz.pose.position = goal.pose.position;        //Aktuális pozíció frissítése

      goal.header.frame_id="map";
      goal.pose = robot1.ut[index];

      ROS_INFO("CEl #%d:\t x: %lf\t y: %lf",index+1, goal.pose.position.x, goal.pose.position.y);
      index++;                                          //Célpozíció kiírása

      utvonal.points.push_back(elozopoz);               //Előzőleg elért pont listához adva
      utvonal.points.push_back(aktpoz.pose.position);   //Most elért pont listához adása
      pub_marker.publish(utvonal);                      //Bejárt útvonal publikálása

      pub_goal.publish(goal);                           //Célpozíció kiküldése
      robot1.celbaert= 0;                               //Elindulás jelzése
    }



  pub_map_update.publish(robot1.terkep_uj);

  //pub_map.publish(robot1.terkep_uj);

  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}