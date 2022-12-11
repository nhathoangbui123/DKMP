// Wifi Credentials
#define SSID "NhatHoang"
#define PASSWORD "01217818548"

// Read readme.md to properly configure api key and authentication

// create a new api key and add it here 
#define API_KEY "AIzaSyCliExrFC2YV5xentcyASfs5oU5O-HP3UA"
// Copy your firebase real time database link here 
#define DATABASE_URL "https://dkmp-4238b-default-rtdb.asia-southeast1.firebasedatabase.app/"  

#define USER_EMAIL "esp32@gmail.com"   // This gmail does not exist outside your database. it only exists in the firebase project as a user
#define USER_PASSWORD "123456"      // Dont add your gmail credentials. Setup users authentication in your Firebase project first

typedef struct 
{
   int ID;//node id
   float CO;//CO
   float GAS;//GAS
   float H; //Humi
   float T; //Temp
   float D; //Dust2.5
   float D10; //Dust10
   int hour; 
   int min;
}fb_data_t;

typedef struct 
{
   float CO;//CO
   float GAS;//GAS
   float H; //Humi
   float T; //Temp
   float D; //Dust2.5
   float D10; //Dust10
   int fan;
   int window;
   char mode[10];

}fb_get_data;

fb_data_t data_get;
fb_get_data data_node1;
fb_get_data data_node2;