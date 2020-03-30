
#include <function.h>

void setup()
{
    INIT();
    setCoupePosition(2000);  //Home
    delay(2000);
    setCoupePosition(-370);
    Serial.println(postionEncodeur);
}

void loop()
{

    //Avencer(VITESSE_AVENCE);
    //checkObstacle();
    

    delay(200);

}