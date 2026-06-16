#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/supervisor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define TIME_STEP 64
#define NUM_ESTADOS 12  //4regiões, 3 movimentos
#define NUM_ACOES 5
#define ALPHA 0.1       //taxa aprendizado
#define GAMMA 0.95      //desconto futuro

double Q[NUM_ESTADOS][NUM_ACOES] = {0};

typedef enum  {FRENTE, DIREITA, ESQUERDA, ROTACIONAR, TRAS,}acao;
typedef enum  {CENTRO, MEIO, BORDA, FORA,}regiao;
typedef enum  {NORMAL, LENTO, BLOQUEADO,}movimento;
//typedef struct{double x; double y; double theta;}coordenadas;
 
void   executar_acao           (acao acao_atual, WbDeviceTag motor_esquerdo, WbDeviceTag motor_direito);
double distancia_centro        (double x, double y);
movimento classificar_movimento(acao acao, double dL, double dR);
regiao achar_regiao            (double distancia);
char*  Regiao                  (regiao r);
int estado_id                  (regiao r, movimento m);
void atualizar_q               (int estado_ant, acao acao_ant, int estado_atual, int recompensa);
int escolher_acao              (int estado);
void salvar_q_table            (char *arquivo);
void carregar_q_table          (char *arquivo);
void resetar_ambiente();
void imprimir_q_table();

WbNodeRef robo, caixa;
WbFieldRef robo_trans, robo_rot, caixa_trans;
double epsilon = 1.0;

int main(int argc, char **argv){
  wb_robot_init();
  srand(time(NULL));
  carregar_q_table("qtable.bin");
  
  int    tempo_decorrido = 0;
  double left_prev       = 0;
  double right_prev      = 0;

  WbDeviceTag motor_esquerdo = wb_robot_get_device("motor esq");
  WbDeviceTag motor_direito  = wb_robot_get_device("motor dir");
  WbDeviceTag encoder_esq    = wb_robot_get_device("encoder esq");
  WbDeviceTag encoder_dir    = wb_robot_get_device("encoder dir");
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag imu = wb_robot_get_device("imu");
  
  robo   = wb_supervisor_node_get_from_def("DEMOLIDOR");
  caixa  = wb_supervisor_node_get_from_def("CAIXA");
  
  robo_trans  = wb_supervisor_node_get_field(robo, "translation");
  robo_rot    = wb_supervisor_node_get_field(robo, "rotation");
  caixa_trans = wb_supervisor_node_get_field(caixa, "translation");
  
  wb_inertial_unit_enable(imu, TIME_STEP);
  wb_position_sensor_enable(encoder_esq, TIME_STEP);
  wb_position_sensor_enable(encoder_dir, TIME_STEP);
  wb_gps_enable(gps, TIME_STEP);
  //motor dc comum
  wb_motor_set_position(motor_esquerdo, INFINITY);
  wb_motor_set_position(motor_direito, INFINITY);
  
  acao acao_anterior;
  acao acao_atual     = FRENTE;
  int estado_anterior = -1;
  
  int episodios      = 0;
  int recompensa_tot = 0;
  int vitorias       = 0;
  int derrotas       = 0;
  
  while (wb_robot_step(TIME_STEP) != -1) {
    tempo_decorrido += TIME_STEP;
    
    const double *pos = wb_gps_get_values(gps);
    double left = wb_position_sensor_get_value(encoder_esq);
    double right= wb_position_sensor_get_value(encoder_dir);
    double delta_left  = left  - left_prev;
    double delta_right = right - right_prev;

    left_prev  = left;
    right_prev = right;
    //printf("L=%f R=%f\n",      left, right); fflush(stdout);
    //printf("dL=%f dR=%f\n",    delta_left, delta_right);
    //printf("x=%f y=%f z=%f\n", pos[0], pos[1], pos[2]);
   
    double d_robo = distancia_centro(pos[0], pos[1]);
    regiao regiao = achar_regiao(d_robo);
   
    //printf("d=%.3f regiao=%s\n", d, Regiao(regiao));
    //const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu);
    //double yaw        = rpy[2];
    //printf("yaw=%f\n", yaw);
    
    movimento mov           = classificar_movimento(acao_atual, delta_left, delta_right);
    int estado_atual        = estado_id(regiao, mov);
    const double *pos_caixa = wb_supervisor_field_get_sf_vec3f(caixa_trans);
    double d_caixa          = distancia_centro(pos_caixa[0], pos_caixa[1]);
    
    int recompensa = 0;
    bool acabou    = false;

    if (d_caixa > 0.40){
      recompensa = 100;
      acabou = true;
      vitorias++;
      printf("CAIXA DERRUBADA\n");
    }else if (d_robo > 0.385){
      recompensa = -100;
      acabou = true;
      derrotas++;
      printf("O ROBÔ CAIU\n");
    }else{
      if (acao_atual == FRENTE && mov == BLOQUEADO)recompensa = 5; 
      else if(acao_atual == FRENTE && mov == LENTO)recompensa = 2;
      else if(regiao == BORDA)recompensa = -3;
      else recompensa = -1; 
    }

    recompensa_tot += recompensa;
    
    if(estado_anterior != -1)atualizar_q(estado_anterior, acao_anterior, estado_atual, recompensa);

    if (acabou){
      episodios++;
      if(epsilon > 0.05)epsilon *= 0.995;
      printf("Episódios: %d | Pontuação Total: %d\n", episodios, recompensa_tot);
      printf("Vitórias:  %d | Derrotas:  %d\n\n", vitorias, derrotas);
      resetar_ambiente();
      
      recompensa_tot  = 0;
      estado_anterior = -1;
      acao_atual      = FRENTE;
      
      if(episodios % 50 == 0){
        imprimir_q_table();
        printf("Taxa de vitória: %.2f%%\n", 100.0 * vitorias /(vitorias + derrotas));
        printf("Epsilon: %.3f\n", epsilon);
        salvar_q_table("qtable.bin");
      }
      continue;
    }
   
    acao_atual = escolher_acao(estado_atual);
    executar_acao(acao_atual, motor_esquerdo, motor_direito);
    
    estado_anterior = estado_atual;
    acao_anterior   = acao_atual;
    /*if (regiao == BORDA && acao_atual != TRAS){
      acao_atual = TRAS;
      tempo_decorrido = 0;
    }else if (acao_atual == TRAS){
      if (tempo_decorrido >= 1000){ 
        acao_atual = ROTACIONAR;
        tempo_decorrido = 0;
      }
    }else if (acao_atual == FRENTE && (mov == BLOQUEADO || mov == LENTO)){
      tempo_decorrido = 0; 
      acao_atual = FRENTE;
    }else if(acao_atual == FRENTE && tempo_decorrido > 600){
      int sorteio     = 1 + rand() % 3;
      acao_atual    = acoes[sorteio];
      tempo_decorrido = 0;
    }else{
      if(tempo_decorrido > 500){
        acao_atual = FRENTE;
        tempo_decorrido = 0; 
      }
    }
    executar_acao(acao_atual, motor_esquerdo, motor_direito);*/
  };
  salvar_q_table("qtable.bin");
  wb_robot_cleanup();

  return 0;
}

void executar_acao(acao acao_atual, WbDeviceTag motor_esquerdo, WbDeviceTag motor_direito){
  switch(acao_atual){
    case FRENTE:
      wb_motor_set_velocity(motor_esquerdo,  10.0);
      wb_motor_set_velocity(motor_direito,   10.0);
      break;
     case ROTACIONAR:
       wb_motor_set_velocity(motor_esquerdo, 7.0);
       wb_motor_set_velocity(motor_direito, -7.0);
       break;
     case DIREITA:
       wb_motor_set_velocity(motor_esquerdo,  5.0);
       wb_motor_set_velocity(motor_direito,  10.0);
       break;
     case ESQUERDA:
       wb_motor_set_velocity(motor_esquerdo, 10.0);
       wb_motor_set_velocity(motor_direito,   5.0);
       break;
     case TRAS:
       wb_motor_set_velocity(motor_esquerdo, -10.0);
       wb_motor_set_velocity(motor_direito,  -10.0);
       break;
  }
}

double distancia_centro(double x, double y){
  return sqrt(x*x + y*y);
}

regiao achar_regiao(double d){
  if     (d < 0.128) return CENTRO;
  else if(d < 0.256) return MEIO;
  else if(d < 0.385) return BORDA;
  else               return FORA;
}

char* Regiao(regiao r){
  char*  regioes[ ] = {"CENTRO", "MEIO", "BORDA", "FORA"};
  return regioes[r];
}

movimento classificar_movimento(acao acao, double dL, double dR){
  //pegar modulo
  double media_delta = (fabs(dL) + fabs(dR)) / 2.0;

  if (acao == FRENTE || acao == TRAS) {
    if (media_delta < 0.15) return BLOQUEADO; 
    if (media_delta < 0.45) return LENTO;     
    return NORMAL;                            
  }
  return NORMAL; 
}

void resetar_ambiente(){
  const double pos_inicial_robo[3] = {0.0, 0.1, 0.0435}; 
  const double rot_inicial_robo[4] = {0.0, 0.0, 1.0, 0.0}; 
  
  wb_supervisor_field_set_sf_vec3f (robo_trans, pos_inicial_robo);
  wb_supervisor_field_set_sf_rotation(robo_rot, rot_inicial_robo);

  double angulo = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
  double raio   = sqrt((double)rand() / RAND_MAX) * 0.25;

  double random_x = raio * cos(angulo);
  double random_y = raio * sin(angulo);
  const double pos_inicial_caixa[3] = {random_x, random_y, 0.06}; 
  wb_supervisor_field_set_sf_vec3f(caixa_trans, pos_inicial_caixa);

  wb_supervisor_node_reset_physics(robo);
  wb_supervisor_node_reset_physics(caixa);
}

int estado_id(regiao r, movimento m){
    return (r * 3) + m; 
}

void atualizar_q(int estado_ant, acao acao_ant, int estado_atual, int recompensa){
  double max_q = Q[estado_atual][0];
  for(int a = 1; a < NUM_ACOES; a++){
    if(Q[estado_atual][a] > max_q) max_q = Q[estado_atual][a];
  }
  
  Q[estado_ant][acao_ant] = Q[estado_ant][acao_ant] + ALPHA*(recompensa + GAMMA*max_q - Q[estado_ant][acao_ant]);
}

int escolher_acao(int estado){
  if(((double)rand() / RAND_MAX) < epsilon)return (rand() % NUM_ACOES);

  int melhor = rand() % NUM_ACOES;
  for(int i = 1; i < NUM_ACOES; i++){
    if(Q[estado][i] > Q[estado][melhor]) melhor = i;
  }
  return melhor;
}

void imprimir_q_table(){
  for(int i = 0; i < NUM_ESTADOS; i++){
    printf("Estado %2d : ", i);
    for(int j = 0; j < NUM_ACOES; j++){
      printf("%8.2f ", Q[i][j]);
    }
    printf("\n");
  }
}

void salvar_q_table(char *arquivo){
  FILE *f = fopen(arquivo, "wb");
  if(f == NULL){
    printf("arquivo não encontrado\n");
    return;
  }

  fwrite(Q, sizeof(double), NUM_ESTADOS * NUM_ACOES, f);
  fclose(f);

  printf("tabela salva\n");
}

void carregar_q_table(char *arquivo){
  FILE *f = fopen(arquivo, "rb");
  if(f == NULL){
    printf("arquivo não encontrado\n");
    return;
  }

  fread(Q, sizeof(double), NUM_ESTADOS * NUM_ACOES, f);
  fclose(f);

  printf("tabela carregada.\n");
}