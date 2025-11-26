#include <stdio.h>  //gcc '3D from Scratch.c' -o '3D from Scratch' $(sdl2-config --cflags --libs) -lm
#include <math.h>
#include <float.h>
#include <limits.h>
#include "SDL.h"
#include <stdbool.h>

#define WINDOW_TITLE "Motor 3D básico"
#define WIDTH 800
#define HEIGHT 800


//ALGUNAS OPERACIONES VECTORIALES
float dotProduct(float* Vector1, float* Vector2){

    float resultado = 0;
    for(int i = 0; i < 3; i++){
        resultado += Vector1[i] * Vector2[i];
    }
    return resultado;
}

void restaVectorial(float* Resultado, float* Vector1, float* Vector2){ 
    for(int i = 0; i < 3; i++){
        Resultado[i] = Vector1[i] - Vector2[i];
    }
}

void sumaVectorial(float* Resultado, float* Vector1, float* Vector2){ 
    for(int i = 0; i < 3; i++){
        Resultado[i] = Vector1[i] + Vector2[i];
    }
}

void multiplicacionEscalar(float* Resultado, float* Vector1, float Escalar){ 
    for(int i = 0; i < 3; i++){
        Resultado[i] = Vector1[i] * Escalar;
    }
}

//DEFINICIÓN DE ALGUNAS VARIABLES Y STRUCTS
const unsigned char colorFondo[4] = {0, 0, 0, 255};
unsigned char colorFinal[4]; //Acá se almacenan el RGB para colorear cada pixel de la pantalla
float direccionRayo[3];
const float luzAmbiental = 0.2;
const float distanciaMinima = 0.001; //aka epsilon
unsigned char limiteRecursion = 2;

typedef struct Viewport{
    float origenCamara[3]; 
    float centroCuadroProyeccion[3];
    float altura;
    float anchura;
} Viewport;

typedef struct Esfera{
    float centro[3];
    unsigned char color[4];
    float radio;
    unsigned short int brillo;
    float reflectividad;
} Esfera;

typedef struct fuenteDeLuz
{
    bool tipo; //1 o true para vector (luz unidireccional). false (cero) para punto (luz omnidireccional)
    float ubicacion[3]; //o bien dirección si se trata de una luz unidireccional
    float intensidad;
} fuenteDeLuz;

void reflejarRayo(float* rayo, float* reflejo, float* normal){
    float dotNormalRayo = dotProduct(rayo, normal);
    multiplicacionEscalar(reflejo, normal, 2.0 * dotNormalRayo);
    restaVectorial(reflejo, reflejo, rayo); // - L
}

//Dimensionar los píxeles de la ventana en el cuadro de proyección de la cámara 
void direccionarRayos(Viewport viewport, float* punto, float* direccionRayo) { //Punto ha de corresponderse con la esquina superior izquiera de una de las 
    //casillas en las que se divide el viewport en función del número de píxeles de la ventana
    float vertical = (float)(viewport.altura / HEIGHT);
    float horizontal = (float)(viewport.anchura / WIDTH);

    float componenteHorizontal = punto[0]; 
    float componenteVertical = punto[1]; 
    direccionRayo[0] = componenteHorizontal;
    direccionRayo[1] = componenteVertical;
    direccionRayo[2] = 1.0;
    
}


//Detección de colisión de esferas. Gracias a esta función no hace falta calcular ningún volumen
//Nos devuelve el punto más cercano en el cual un rayo colisiona con algún punto perteneciente a la superficie de alguna de las esferas.
void detectarColisionDeEsferas(Esfera esfera, float* origenCamara, float* direccion, float* colisiones){ //aka IntersectRaySphere

    float OC[3];
    restaVectorial(OC, esfera.centro, origenCamara);
    float a = dotProduct(direccion, direccion);
    float b = -2 * dotProduct(OC, direccion);
    float c = dotProduct(OC, OC) - (esfera.radio * esfera.radio);

    float discriminante = (b * b) - (4 * a * c);

    if(discriminante < 0.0) {
        colisiones[0] = FLT_MAX;
        colisiones[1] = FLT_MAX;
        return;
    }

    colisiones[0] = (-b + sqrt(discriminante)) / (2*a);
    colisiones[1] = (-b - sqrt(discriminante)) / (2*a);

}

float detectarColisionMasCercana(float* direccion, float* colisiones, Esfera* esferas, Esfera** esferaMasCercana, float* punto, float distanciaMinima,
    float distanciaMaxima){ //aka ClosestIntersection

    float colisionMasCercana = FLT_MAX;

    for(int i = 0; i < 4; i++){
        detectarColisionDeEsferas(esferas[i], punto, direccion, colisiones);
        if (colisiones[0] < colisionMasCercana && distanciaMinima < colisiones[0] && colisiones[0] < distanciaMaxima) { 
            colisionMasCercana = colisiones[0];
            *esferaMasCercana = &esferas[i]; 
        }
        if (colisiones[1] < colisionMasCercana && distanciaMinima < colisiones[1] && colisiones[1] < distanciaMaxima) {
            colisionMasCercana = colisiones[1];
            *esferaMasCercana = &esferas[i];
        }
    }
    return colisionMasCercana;
}

float calcularIntensidadLuz(float* punto, float* normal, fuenteDeLuz* fuentesDeLuz, const float luzAmbiental, unsigned short int numDeLuces,
    float* vista, unsigned short int brillo /*aka specular */, const float distanciaMinima/*epsilon*/, Esfera* esferas){ //aka ComputeLighting
    
    float distanciaMaxima; //aka t_max
    float intensidad = luzAmbiental;
    float longitudNormal = sqrt(dotProduct(normal, normal));
    fuenteDeLuz* luzActual;
    Esfera* esferaMasCercana = NULL;
    float colisiones[2];
    float vectorLuz[3]; //aka vec_l
    
    for(int i = 0; i < numDeLuces; i++){
        luzActual = &fuentesDeLuz[i];
        if(luzActual->tipo == true){ //Luz unidireccional infinitamente distante
            for(int k = 0; k < 3; k++) vectorLuz[k] = fuentesDeLuz[i].ubicacion[k];
            distanciaMaxima = FLT_MAX;
        }
        else{ //Luz omnidireccional a una distancia finita
            restaVectorial(vectorLuz,fuentesDeLuz[i].ubicacion, punto);
            distanciaMaxima = 1.0;
        }

    float dotNormalLuz = dotProduct(normal, vectorLuz); //aka n_dot_l

    //determinar sombreado
    float colisionMasCercana = detectarColisionMasCercana(vectorLuz, colisiones, esferas, &esferaMasCercana, punto, 0.001, distanciaMaxima); 

    if(esferaMasCercana != NULL) continue; //Esto solo afecta el brillo, es decir, la reflexión especular cuando lo pongo como if(esferaMasCercana == NULL)
    //

    if(dotNormalLuz > 0.0){
        float longitudVectorLuz = sqrt(dotProduct(vectorLuz, vectorLuz));
        intensidad += luzActual->intensidad * (dotNormalLuz / (longitudVectorLuz)); 
    }


    //Iluminación especular (brillo)
    if(brillo != 0.0){
        float vecLuzReflejada[3] = {0.0, 0.0, 0.0}; //aka vec_r
        multiplicacionEscalar(vecLuzReflejada, normal, 2.0 * dotNormalLuz);
        restaVectorial(vecLuzReflejada, vecLuzReflejada, vectorLuz); // - L
        float dotReflejoVista = dotProduct(vecLuzReflejada, vista); //aka r_dot_v


        if(dotReflejoVista > 0.0){
            float longitudVista = sqrt(dotProduct(vista,vista)); //aka vec_r.length()
            float longitudVecLuzReflejada = sqrt(dotProduct(vecLuzReflejada,vecLuzReflejada));
            float intensidadLuzEspecular = pow(dotReflejoVista / (longitudVecLuzReflejada * longitudVista), brillo);
            if(intensidad + intensidadLuzEspecular > 1.0){
                intensidad = 1.0;
            }
            else intensidad += intensidadLuzEspecular;
        }
    }
}
    
    printf("intensidad %.3f\n", intensidad);
    return intensidad; 
}

void trazarRayos(float* origenRayo, Esfera* esferas, float* direccionRayo, const unsigned char* colorFondo, unsigned char* colorFinalDelPixel,
    const float luzAmbiental, fuenteDeLuz* fuentesDeLuz, const float distanciaMinima, unsigned char limiteRecursion){ 
    
    Esfera* esferaMasCercana = NULL;

    float escalaresDeColision[2]; //Los escalares que, al multiplicar a un vector que va desde el origen de la cámara hacia el punto de proyección
    // en el área del "viewframe" nos dan los puntos de colisión entre los rayos trazados y las 2 colisiones detectadas en la superficie de alguna esfera
    
    float colisionMasCercana = detectarColisionMasCercana(direccionRayo, escalaresDeColision, esferas, &esferaMasCercana, origenRayo, 1, FLT_MAX); //Esto sería todo lo de allá arriba pero llamado desde una función

    if (esferaMasCercana != NULL) {

        float punto[3] = {0.0, 0.0, 0.0};
        multiplicacionEscalar(punto, direccionRayo, colisionMasCercana);

        if(punto[0] != 0.0 || punto[1] != 0.0 || punto[2] != 0.0) sumaVectorial(punto, punto, origenRayo);

        float normal[3] = {0.0, 0.0, 0.0};
        restaVectorial(normal, punto, esferaMasCercana->centro);
        float escalarNormalizador = 1.0 / sqrt(dotProduct(normal, normal));
        multiplicacionEscalar(normal, normal, escalarNormalizador);
        

        float longitudNormal = sqrt(dotProduct(normal, normal));
        printf("longitud de la normal %.3f\n", longitudNormal);

        float intensidadFinal = 0.0;

        float vista[3] = {0.0, 0.0, 0.0};
        multiplicacionEscalar(vista, direccionRayo, -1);

        for (int k = 0; k < 4; k++) { 
            intensidadFinal = calcularIntensidadLuz(punto, normal, fuentesDeLuz, luzAmbiental, 2, vista, esferaMasCercana->brillo, distanciaMinima, esferas);  
            colorFinalDelPixel[k] = esferaMasCercana->color[k] * intensidadFinal;
        }

        float reflectividad = esferaMasCercana->reflectividad;
        if(limiteRecursion == 0 || esferaMasCercana->reflectividad == 0) return;

        float reflejoFinal[4] = {0.0, 0.0, 0.0, 0.0};
        reflejarRayo(vista, reflejoFinal ,normal);

        // 2. NUEVO ORIGEN Y DIRECCIÓN para el rayo reflejado
        float nuevoOrigen[3];
        // Movemos el origen un poco sobre la normal (epsilon) para evitar chocar consigo mismo.
        multiplicacionEscalar(nuevoOrigen, normal, distanciaMinima);
        sumaVectorial(nuevoOrigen, punto, nuevoOrigen);

        unsigned char colorReflejado[4] = {0, 0, 0, 0};       
        trazarRayos(nuevoOrigen, esferas, reflejoFinal, colorFondo, colorReflejado, luzAmbiental, fuentesDeLuz, distanciaMinima, limiteRecursion - 1);
        //multiplicacionEscalar(colorReflejado, colorReflejado, (1 - reflectividad));
        for (int k = 0; k < 4; k++) {
            colorReflejado[k] *= reflectividad;
            colorFinalDelPixel[k] *= (1 - reflectividad);
            if((colorFinalDelPixel[k] + colorReflejado[k]) > 255) colorFinalDelPixel[k] = 255;
            else colorFinalDelPixel[k] += colorReflejado[k];
        }
    } else {
        // Si no, usamos el color de fondo.
        for (int k = 0; k < 4; k++) {
            colorFinalDelPixel[k] = colorFondo[k];
        }
    }

}

//DEFINICIÓN DE LA VENTANA Y TODOS LOS ELEMENTOS DE SDL PARA EL RENDERIZADO
typedef struct Motor{
    SDL_Window* window;
    SDL_Renderer* renderer;
} Motor;

bool inicMotor(Motor* motor){
    if(SDL_Init(SDL_INIT_EVERYTHING)){
        fprintf(stderr, "Error en la iniciación de SDL %s\n", SDL_GetError());
        return true;
    }
    motor->window = SDL_CreateWindow(WINDOW_TITLE, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIDTH, HEIGHT, 0);
    if(!motor->window){
        fprintf(stderr, "Error en la creación de ventana %s\n", SDL_GetError());
        return true;    
    }

    motor->renderer = SDL_CreateRenderer(motor->window, -1, 0);
    if(!motor->renderer){
        fprintf(stderr, "Error en la creación del renderizador %s\n", SDL_GetError());
        return true;    
    }

    return false;
}

void limpiarMemoria(Motor* motor, int exitStatus){
    SDL_DestroyRenderer(motor->renderer);
    SDL_DestroyWindow(motor->window);
    exit(exitStatus);

}

int main(){

    //Preparamos la escena
    Esfera esferas[4] ={ {{0, -1, 3}, {255, 0, 0, 255}, 0.5, 500, 0.2},
                        {{-1.5, 0, 4}, {0, 255, 0, 255}, 0.5, 10, 0.2},
                        {{1.5, 0, 4},  {0, 0, 255, 255}, 0.5, 500, 0.2},
                        {{0, -5001, 0},  {255, 255, 0, 255}, 5000, 1000, 0.6}
    };

    fuenteDeLuz fuentesDeLuz[2] = {
        { true, {2.0, 1.0, 0.0}, 0.6 }, //Vector o fuente de luz unidireccional
        { false, {1.0, 4.0, 4.0}, 0.2 } //Punto o fuente de luz omnidireccional

    };


    Viewport camara = { {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 1.0, 1.0 };

    float puntoRayo[3] = {-0.5, 0.5, 1.0}; //Situado en la esquina superior izquierda del cuadro de proyección de la cámara


    Motor motor3D = {
        .window = NULL,
        .renderer = NULL,
    };

    if(inicMotor(&motor3D)){
        limpiarMemoria(&motor3D, EXIT_FAILURE);
        puts("Algo salió mal");
    }

    while(true){
        SDL_Event event;
        while(SDL_PollEvent(&event)){
            if(event.type == SDL_QUIT){
                limpiarMemoria(&motor3D, EXIT_SUCCESS);
                break;
            }
        }
        SDL_RenderClear(motor3D.renderer);
        puntoRayo[0] = -0.5;
        puntoRayo[1] = 0.5;
        puntoRayo[2] = 1.0;
        //Bucle de renderizado
        for(int y = 0; y < HEIGHT; y++){
            for(int x = 0; x < WIDTH; x++){
                direccionarRayos(camara, puntoRayo, direccionRayo);
                trazarRayos(camara.origenCamara, esferas, direccionRayo, colorFondo, colorFinal, luzAmbiental, fuentesDeLuz, distanciaMinima, limiteRecursion);
                SDL_SetRenderDrawColor(motor3D.renderer, colorFinal[0], colorFinal[1], colorFinal[2], colorFinal[3]);
                SDL_RenderDrawPoint(motor3D.renderer, x, y);

                puntoRayo[0] += camara.altura/WIDTH;
            }
            puntoRayo[1] -= camara.altura/HEIGHT;
            puntoRayo[0] = -0.5;
        }
        SDL_RenderPresent(motor3D.renderer);

    }

    limpiarMemoria(&motor3D, EXIT_SUCCESS);
    puts("Todo bien!");

    return 0;
}