#include <function.h>

void setup()
{
    delay(2000);
    INIT();
    delay(5000);
    Avencer(VITESSE_AVENCE);
    
}

void loop()
{ 
    checkObstacle();
    delay(100);

}
