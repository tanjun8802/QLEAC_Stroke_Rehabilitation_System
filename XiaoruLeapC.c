// /∗ C o p y r i g h t (C) 2012−2017 U l t r a l e a p L im i t e d . A l l r i g h t s r e s e r v e d .
//  ∗
//  ∗ Use o f t h i s c o de i s s u b j e c t t o t h e term s o f t h e U l t r a l e a p SDK
// greemen t
//  ∗ a v a i l a b l e a t h t t p s : / / c e n t r a l . l e a pm o t i o n . com/ ag reeme n t s /SdkAgreemen t
//  n l e s s
//  ∗ U l t r a l e a p h a s s i g n e d a s e p a r a t e l i c e n s e agreemen t w i t h you or your
//  ∗ o r g a n i s a t i o n .
//  ∗
//  ∗/

// Code modified by Xiarou Sun, xs1a12@soton.ac.uk, 2023
// Modified to send data in JSON format: https://www.geeksforgeeks.org/cjson-json-file-write-read-modify-in-c/

 # define _CRT_SECURE_NO_WARNINGS // remove s p r i n t f e r r o r

 #include <stdio.h>
 #include <stdlib.h>
 #include "LeapC.h"
 #include "ExampleConnection.h"


 #define _USE_MATH_DEFINES
 #include <math.h> //For u s i n g v e c t o r and n o rm a l i s i n g
 #include <winsock2.h> // s o c k e t s (windows equivalent)
 #include <time.h>  // c l o c k


 clock_t oldtime ;

 WSADATA WinSockData ;
 int sockfd = 0;
 struct sockaddr_in serv_addr ; // TCPServerAddr ;
 char SendBuffer [512] = "POSE 1, 3, 5, 7, 5, 6, 7, 8, 9, 10, 11, 12, 13 \r\n";

 FILE *fptr;


 # pragma comment(lib,"ws2_32.lib") //Winsock L i b r a r y

 static LEAP_CONNECTION* connectionHandle;
 


 double KVx [23] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
 double KVy [23] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
 double KVz [23] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };
 double ja [18] = { 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 };

 ///∗∗ C a l l b a c k f o r when t h e c o n n e c t i o n o pen s . ∗/
 static void OnConnect () {
 printf ("Connected . \n") ;
 }

 // /∗∗ Callback for when a device is found. ∗/
 static void OnDevice ( const LEAP_DEVICE_INFO *props ) {
 printf ("Found d e v i c e %s . \n", props -> serial ) ;
 }

 //MUST DECLARE BEFORE THEY ARE USED
 double* Cross ( double X1 , double Y1 , double Z1 , double X2 , double Y2 ,double Z2 ){
 double vecres[3] = { ( Y1 * Z2 ) - ( Z1 * Y2 ) , ( Z1 * X2 ) - ( X1 * Z2 ) , (X1 * Y2 ) - ( Y1 * X2 ) };
 return vecres;
 }


 double getAngle ( double X1 , double Y1 , double Z1 , double X2 , double Y2 , double Z2 ){
 double angle = 0;
 if (!((( X1 == 0) && ( Y1 == 0) && ( Z1 == 0) ) || (( X2 == 0) && ( Y2 == 0) && ( Z2 == 0) ) ) ){
 angle = acos (( X1 * X2 + Y1 * Y2 + Z1 * Z2 ) / ( sqrt ( X1 * X1 + Y1 * Y1 + Z1 * Z1 ) * sqrt ( X2 * X2 + Y2 * Y2 + Z2 * Z2 ) ) ) ; //Cosine rule based on 3D bone segment lengths
 }
 return angle * 180 / M_PI ; // pow(X1 , 2 ) ;
 }
// /∗ C a l l b a c k f o r when a frame o f t r a c k i n g d a t a i s a v a i l a b l e . ∗/
 static void OnFrame ( const LEAP_TRACKING_EVENT *frame ) {
 ///∗
//  if ( frame−>info.frameid % 60 == 0 )
//  printf(”Frame %lli with %i hands . \n” , ( long long int ) frame−>info.
// frameid , frame−>nHands ) ;
 //∗/

 for ( uint32_t h = 0; h < frame -> nHands ; h ++) {
 LEAP_HAND* hand = &frame -> pHands [ h ];
 //MIRROR r i g h t t o l e f t : s im p l e GOSAIL method f o r g e t t i n g c o n s i s t e n t i n t e r p r e t a t i o n s
 double m = 1; 
 if ( hand -> type == eLeapHandType_Left ) { m = -1;
}


 // l e t ’ s j u s t l o o k a t l e f t hand
 if ( hand -> type == eLeapHandType_Right) // eLeapHandType Le f t )
 {
 // el b ow
 KVx [0] = hand -> arm.prev_joint.x ; KVy [0] = hand -> arm.prev_joint.y ; KVz [0] = hand -> arm.prev_joint.z ;
 // w r i s t
 KVx [1] = hand -> arm.next_joint.x ; KVy [1] = hand -> arm.next_joint.y ; KVz [1] = hand -> arm.next_joint.z;
 // palm p o s i t i o n
 KVx [2] = hand -> palm.position.x ; KVy [2] = hand -> palm.position.y ; KVz [2] = hand -> palm.position.z ;
 // D i s t a l en d s o f b one s f o r e ac h d i g i t
 for (int f = 0; f < 5; f ++) {
 LEAP_DIGIT finger = hand -> digits [ f ]; // I t h i n k : thumb, index , m i d dle , r i n g , p i n k y
 for (int b = 0; b < 4; b ++) {
 LEAP_BONE bone = finger.bones [ b ];
 KVx [3 + f * 4 + b ] = bone.next_joint.x ;
 KVy [3 + f * 4 + b ] = bone.next_joint.y ;
 KVz [3 + f * 4 + b ] = bone.next_joint.z ;
 }
 }

 // FROM GOSAIL (Form1)
 double* wristaxis = Cross ( KVx [0] - KVx [1] , KVy [0] - KVy [1] , KVz [0] - KVz [1] , ( KVx [1] - KVx [2]) , KVy [1] - KVy [2] , KVz [1] - KVz [2]) ;
 double check = wristaxis [0] * ( KVx [7] - KVx [19]) + wristaxis [1] * ( KVy [7] - KVy [19]) + wristaxis [2] * ( KVz [7] - KVz [19]) ;

 //// LH addition 11/01/24 - used to invert (index) MCP angle during hyperextension
 // Currently set for right hand and m might be in slightly the wrong place for left hand
//double* jointaxis = Cross(KVx[1]-KVx[7], KVy[1]-KVy[7], KVz[1]-KVz[7], m*(KVx[7]-KVx[8]), KVy[7]-KVy[8], KVz[7]-KVz[8]);
double X1=KVx[1]-KVx[7]; double Y1=KVy[1]-KVy[7]; double Z1=KVz[1]-KVz[7]; double X2=(KVx[7]-KVx[8]); double Y2=KVy[7]-KVy[8]; double Z2=KVz[7]-KVz[8];

double jointaxis[3] = {( Y1 * Z2 ) - ( Z1 * Y2 ) , ( Z1 * X2 ) - ( X1 * Z2 ) , (X1 * Y2 ) - ( Y1 * X2 )};
// double* PPperp = Cross(jointaxis[0], jointaxis[1], jointaxis[2], m*(KVx[7]-KVx[8]), KVy[7]-KVy[8], KVz[7]-KVz[8]); // Axis perpendicular to proximal phalange
// double* palmPerp = Cross(KVx[1]-KVx[7], KVy[1]-KVy[7], KVz[1]-KVz[7], m*(KVx[7]-KVx[19]), KVy[7]-KVy[19], KVz[7]-KVz[19]); //Axis perpendicular to the palm
double checkMCP = (KVx[7]-KVx[19])*jointaxis[0]+(KVy[7]-KVy[19])*jointaxis[1]+(KVz[7]-KVz[19])*jointaxis[2];
//double checkMCP = palmPerp[0]*PPperp[0]*m+palmPerp[1]*PPperp[1]+palmPerp[2]*PPperp[2];
 ////

 ja [0] = getAngle ( KVx [0] - KVx [1] , KVy [0] - KVy [1] , KVz [0] - KVz [1] , ( KVx [1] - KVx [2]) , KVy [1] - KVy [2] , KVz [1] - KVz [2]) ;
 if (check*m>0) {ja [0] = - ja [0];}
 ja [1] = getAngle ( m * ( KVx [7] - KVx [19]) , KVy [7] - KVy [19] , KVz [7] - KVz [19] , KVx [0] - KVx [1] , KVy [0] - KVy [1] , KVz [0] - KVz [1]) - 90; // w r i s t abb / add ( a s GOSAIL embedded )
 ja [2] = getAngle ( m * ( KVx [1] - KVx [19]) , KVy [1] - KVy [19] , KVz [1] - KVz [19] , m * ( KVx [19] - KVx [20]) , KVy [19] - KVy [20] , KVz [19] - KVz [20]) ; // l i t t l e 1
 ja [3] = getAngle ( m * ( KVx [19] - KVx [20]) , KVy [19] - KVy [20] , KVz [19] - KVz [20] , m * ( KVx [20] - KVx [21]) , KVy [20] - KVy [21] , KVz [20] - KVz [21]) ; // l i t t l e 2

ja [4] = getAngle ( m * ( KVx [1] - KVx [15]) , KVy [1] - KVy [15] , KVz [1] - KVz [15] , m * ( KVx [15] - KVx [16]) , KVy [15] - KVy [16] , KVz [15] - KVz [16]) ; // r i n g 1
 ja [5] = getAngle ( m * ( KVx [15] - KVx [16]) , KVy [15] - KVy [16] , KVz [15] - KVz [16] , m * ( KVx [16] - KVx [17]) , KVy [16] - KVy [17] , KVz [16] - KVz [17]) ; // r i n g 2
 ja [6] = getAngle ( m * ( KVx [1] - KVx [11]) , KVy [1] - KVy [11] , KVz [1] - KVz [11] , m * ( KVx [11] - KVx [12]) , KVy [11] - KVy [12] , KVz [11] - KVz [12]) ; // m i d dl e 1
 ja [7] = getAngle ( m * (KVx[11]-KVx[12]), KVy[11]-KVy[12], KVz[11]-KVz[12], m * (KVx[12]-KVx[13]), KVy[12]-KVy[13], KVz[12]-KVz[13]) ; // m i d dl e 2
 ja [8] = getAngle ( m*(KVx[1]-KVx[7]), KVy[1]-KVy[7], KVz[1]-KVz[7], m*(KVx[7]-KVx[8]), KVy[7]-KVy[8], KVz[7]-KVz [8]); // i n d e x 1
 if(checkMCP*m<0){ja [8] = - ja [8];}
 ja [9] = getAngle ( m*(KVx[7]-KVx[8]), KVy[7]-KVy[8], KVz[7]-KVz[8], m *(KVx[8]-KVx[9]), KVy[8]-KVy[9], KVz[8]-KVz [9]) ; // i n d e x 2
 ja [10] = getAngle ( m * ( KVx [1] - KVx [3]) , KVy [1] - KVy [3] , KVz [1] - KVz [3] , m * ( KVx [3] - KVx [4]) , KVy [3] - KVy [4] , KVz [3] - KVz [4]) ; // thumb 1
 ja [11] = getAngle ( m * ( KVx [3] - KVx [4]) , KVy [3] - KVy [4] , KVz [3] - KVz [4] , m * ( KVx [4] - KVx [5]) , KVy [4] - KVy [5] , KVz [4] - KVz [5]) ; // thumb 2
 ja [12] = getAngle ( m * ( KVx [4] - KVx [5]) , KVy [4] - KVy [5] , KVz [4] - KVz [5] , m * ( KVx [5] - KVx [6]) , KVy [5] - KVy [6] , KVz [5] - KVz [6]) ; // thumb 3

 ja [0] = - ja [0]+90; // 19122022 XIAORU CHANGE
 
 printf (" wrist flex: %4.4f; index 1: %4.4f; index 2: %4.4f \n", ja [0], ja [8], ja [9]);

//   fptr=fopen("matlab_log.csv", "a");
//   // joint angle/ locations
//   fprintf(fptr, "%4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, ", ja [8], KVx[1], KVy[1], KVz[1], KVx[7], KVy[7], KVz[7], KVx[8], KVy[8], KVz[8], KVx[19], KVy[19], KVz[19]);
//  //Joint axis vectors
//   fprintf(fptr, "%p, %p, %p, %p, %p, %p, %p, %p, %p, %p, ", jointaxis[0], jointaxis[1], jointaxis[2], PPperp[0], PPperp[1], PPperp[2], palmPerp[0], palmPerp[1], palmPerp[2], checkMCP);
//   //Extra vectors for checking if cross function works
//   fprintf(fptr, "%p, %p, %p, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f, %4.4f \n", wristaxis[0], wristaxis[1], wristaxis[2], KVx[2], KVy[2], KVz[2], KVx[0], KVy[0], KVz[0]);
  
//   fclose(fptr);
 
 // p r i n t f (” l i t t l e 1 , l i t t l e 2 (%f , %f ) . \n” , j a [ 2 ] , j a [ 3 ] )

 // p r i n t f (” r i n g 1 , r i n g 2 (%f , %f ) . \n” , j a [ 4 ] , j a [ 5 ] ) ;
 // p r i n t f (” m i d dl e 1 , m i d dl e 2 (%f , %f ) . \n” , j a [ 6 ] , j a [ 7 ] );
 // p r i n t f (” i n d e x 1 , i n d e x 2 (%f , %f ) . \n” , j a [ 8 ] , j a [ 9 ] ) ;
 // p r i n t f (” thumb 1 , thumb 2 , thumb 3 (%f , %f , %f ) . \n” , j a [ 1 0 ] , j a [ 1 1 ] , j a [ 1 2 ] ) ; / / USE thumb1 and thumb3 a s t h e y move most

 //SEND TO myRio IF 50ms has elapsed

//cJSON *json = cJSON_CreateObject(); //Create Json object


  int msec = ( clock () - oldtime ) * 1000 / CLOCKS_PER_SEC ;
 if ( msec >= 50)
 { 

 sprintf ( SendBuffer , "POSE %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f \r\n", ja [0] , ja [1] , ja [2] , ja [3] , ja [4] , ja [5] , ja [6] , ja [7] ,ja [8] , ja [9] , ja [10] , ja [11] , ja [12]) ;

//sprintf ( SendBuffer , "2,3,4,5,6,7,8,9,10,11,12,13,14 \r\n") ;
//printf(SendBuffer);

 if (send( sockfd , SendBuffer , sizeof ( SendBuffer ) , 0) == SOCKET_ERROR ) { //Send data to client (myRio)
 printf ("CLIENT: Send Failed, Error: %d\n", WSAGetLastError () ) ;
 }
 else printf ("CLIENT: Message Sent! \n");


//Free JSON string
//  cJSON_free(json_str); 
//    cJSON_Delete(json);

oldtime = clock () ;
 } 
 }
} //End of onFrame() function

 // s e e h t t p s : / / d oc s . u l t r a l e a p . com/ t r a c k i n g −a p i / gr ou p / g r o u p s t r u c t s .h tml
 }

 static void OnImage (const LEAP_IMAGE_EVENT *image) {
 // printf("Image detected");
//  /∗
//  p r i n t f (” Image % l l i => L e f t : %d x %d ( bpp=%d ) , R i g h t : %d x %d ( bpp=%d ) \n” ,
//  ( l o n g l o n g i n t ) image−>i n f o . f r am e i d ,
//  image−>image [ 0 ] . p r o p e r t i e s . w i d t h , image−>image [ 0 ] . p r o p e r t i e s . h e i g h t ,image−>image [ 0 ] . p r o p e r t i e s . bpp ∗8 ,
//  image−>image [ 1 ] . p r o p e r t i e s . w i d t h , image−>image [ 1 ] . p r o p e r t i e s . h e i g h t ,image−>image [ 1 ] . p r o p e r t i e s . bpp ∗8 ) ;
//  ∗/
 }

 static void OnLogMessage ( const eLeapLogSeverity severity , const int64_t
timestamp ,
 const char *message ) {
 const char *severity_str ;
 switch ( severity ) {
 case eLeapLogSeverity_Critical :
 severity_str = "Critical";
 break ;
 case eLeapLogSeverity_Warning :
 severity_str = "Warning";
 break ;
 case eLeapLogSeverity_Information :
 severity_str = "Info";
 break ;
 default :
 severity_str = "";
 break ;
 }
 printf ("[%s ][%lli ] %s \n", severity_str , ( long long int) timestamp ,message ) ;
 }

 static void *allocate ( uint32_t size , eLeapAllocatorType typeHint , void *state ) {
 void *ptr = malloc ( size ) ;
return ptr ;
 }

 static void deallocate ( void *ptr , void *state ) {
 if (!ptr )
 return ;
 free ( ptr ) ;
 }
void OnPointMappingChange ( const LEAP_POINT_MAPPING_CHANGE_EVENT *change ) {
 if (!connectionHandle )
 return ;


  uint64_t size = 0;

  //No longer supported so calling these has no effect
//  if ( LeapGetPointMappingSize (*connectionHandle , &size ) !=eLeapRS_Success || ! size )
//  return ;

 LEAP_POINT_MAPPING *pointMapping = ( LEAP_POINT_MAPPING *) malloc (( size_t )size ) ;
 if (! pointMapping )
 return ;

// Again no longer supported
//  if ( LeapGetPointMapping (* connectionHandle , pointMapping , &size ) == eLeapRS_Success &&pointMapping -> nPoints > 0) {
//  printf ("Managing %u points as of frame %lld at %lld \n", pointMapping
// -> nPoints , ( long long int) pointMapping -> frame_id , ( long long int)
// pointMapping -> timestamp ) ;
//  }

 free (pointMapping) ;
 }

 void OnHeadPose ( const LEAP_HEAD_POSE_EVENT *event ) {
 printf ("Head pose : \n") ;
 printf (" Head position (%f , %f , %f ) . \n",
 event -> head_position .x ,
 event -> head_position .y ,
 event -> head_position . z ) ;
 printf ("Head orientation (%f , %f , %f , %f ) . \n",
 event -> head_orientation .w ,
 event -> head_orientation .x ,
 event -> head_orientation .y ,
 event -> head_orientation . z ) ;
 printf ("Head linear velocity (%f , %f , %f ) . \n",
 event -> head_linear_velocity .x ,
 event -> head_linear_velocity .y ,
 event -> head_linear_velocity . z ) ;
 printf ("Head angular velocity (%f , %f , %f ) . \n",
event -> head_angular_velocity .x ,
 event -> head_angular_velocity .y ,
 event -> head_angular_velocity . z ) ;
 }
int main (int argc , char** argv ) {
   fptr=fopen("matlab_log.csv", "w");
  fprintf(fptr, "");
 fclose(fptr);
 // S e t c a l l b a c k f u n c t i o n p o i n t e r s
 ConnectionCallbacks.on_connection = &OnConnect;
 ConnectionCallbacks.on_device_found = &OnDevice;
 ConnectionCallbacks.on_frame = &OnFrame;
ConnectionCallbacks.on_image = &OnImage;
 ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
 ConnectionCallbacks.on_log_message = &OnLogMessage;
 ConnectionCallbacks.on_head_pose = &OnHeadPose;




 // ///////////////////////////////////////////////////
 // https://stackoverflow.com/questions/4638604/where-does-one-get-the-sys-socket-h-header-source-file
// https://www.educative.io/answers/how-to-implement-tcp-sockets-in-c
 // https://www.binarytides.com/winsock-socket-programming-tutorial/
 // h t t p : / / c o l l e c t i v e s o l v e r . com/41720/how−to−c r e a t e −tc p−c l i e n t −s e r v e r −s o c k e t −commun ica t ion−on−windows−in−c
 // https://stackoverflow.com/questions/15660203/inet-pton-identifier-not-found BEST REF THAT SOLVED PROBLEM


 oldtime = clock() ;

  //STEP−1 WSASatrtUp
 if (WSAStartup ( MAKEWORD (2 , 2) , & WinSockData ) != 0) {
 printf ("CLIENT: WSAStartUp Failed \n") ;
 }
 else printf ("CLIENT: WSAStartUp Success \n") ;

 // STEP−2 Create Socket (and check it was been correctly setup) - sockfd is the sock descriptor used to refer to the socket
 if (( sockfd = socket ( AF_INET , SOCK_STREAM , IPPROTO_TCP ) ) ==
INVALID_SOCKET ) {
 printf ("CLIENT: TCP Client: Create Socket Failed, Error: %d\n",WSAGetLastError () ) ;
 }
 else printf ("CLIENT: TCP Client: Create Socket Success \n") ;

 memset (&serv_addr, '0' , sizeof(serv_addr)); //Clean server buffer

 // STEP−3 Fill TCPServerAddr (initialise client address- this should be the same as at the server side
 serv_addr.sin_family = AF_INET; // AF INET ;
 serv_addr.sin_port = htons(6340); //Server port number
 //inet_pton (AF_INET , "152.78.242.85" , &serv_addr.sin_addr); // ethernet
 //inet_pton (AF_INET , "172.22.11.2" , &serv_addr.sin_addr); // myRio IP address
//inet_pton (AF_INET , "10.9.181.23" , &serv_addr.sin_addr); // rpi IP address (eduroam)
inet_pton (AF_INET , "152.78.242.147" , &serv_addr.sin_addr); // rpi IP address (ethernet)



 // STEP−4 Connect to Server
 if ( connect(sockfd , (struct sockaddr*) &serv_addr , sizeof(serv_addr) )== SOCKET_ERROR ) {
 printf ("CLIENT: Connection Failed, Error : %d\n", WSAGetLastError () );
 }
 else printf ("CLIENT: Connection Success \n") ;

 // STEP−5 Recv Message From Server //We're not receiving messages from myRio - data transfer only in one direction
// i f ( r e c v ( TCPClientSocket , RecvBuffer , sizeof(RecvBuffer) , 0 ) == SOCKET ERROR) {
 // p r i n t f (”CLIENT: Receive Message Failed , Error : %d \n” , WSAGetLastError ( ) ) ;
 // }
 // p r i n t f (”CLIENT: Received Message From Server : %s \n” , RecvBuffer);

 // STEP−6 Send Message to Server
 if (send( sockfd , SendBuffer , sizeof ( SendBuffer ) , 0) == SOCKET_ERROR ) {
 printf ("CLIENT: Send Failed, Error : %d\n", WSAGetLastError () ) ;
 }
 else printf ("CLIENT: Message Sent! \n");



 connectionHandle = OpenConnection() ;
 {
 LEAP_ALLOCATOR allocator = { allocate , deallocate , NULL };
 LeapSetAllocator (*connectionHandle , &allocator ) ;
 }
 LeapSetPolicyFlags (*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0) ;

 printf ("Press Enter to exit program. \n") ;
 getchar() ;

  // STEP−7 Close Socket
 if ( closesocket( sockfd ) == SOCKET_ERROR ) {
 printf ("CLIENT: Close Socket Failed, Error: %d\n", WSAGetLastError
() ) ;
 }
 else printf ("CLIENT: Close Socket Success \n") ;

 // STEP−8 Clean
 if ( WSACleanup () == SOCKET_ERROR ) {
 printf ("CLIENT: WSACleanup Failed, Error: %d\n", WSAGetLastError () );
 }
 else printf ("CLIENT: WSACleanup Success \n") ; 
 


  CloseConnection();
  DestroyConnection(); //This is probably important but currently can't find the function it's referencing

  return 0;
 }
