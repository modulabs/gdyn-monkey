#define GRAVITY 9.81

struct Link
{
  float m;
  float l;
  float lc;
  float Ic;
  float b;
  float I;

  Link() {}
  Link(float _m, float _l, float _lc, float _Ic, float _b)
    : m(_m), l(_l), lc(_lc), Ic(_Ic), b(_b), I(_Ic+_m*_lc*_lc) {}  
};

class Acrobot
{
  private:
    Link link1;
    Link link2;

    float vecDesQ[2];
    float vecDesQdot[2];
    float vecDesQddot[2];

    float matH[2][2];
    float matC[2][2];

    float vecC[2];
    float VecG[2];
    float VecB[2];
    float VecP[2];

    float alpha;
    float kp;
    float kd;

  private: 
    void calcManipulatorDynamics(float *vecQ, float *vecQdot)
    {
      float q1 = vecQ[0];
      float q2 = vecQ[1];
      float q1_dot = vecQdot[0];
      float q2_dot = vecQdot[1];

      matH[0][0] = link1.I + link2.I + link2.m * link1.l * link1.l + 2.0f * link2.m * link1.l * link2.lc * cos(q2);
      matH[0][1] = link2.I + link2.m * link1.l * link2.lc * cos(q2);
      matH[1][0] = matH[0][1];
      matH[1][1] = link2.I;

      matC[0][0] = -2.0f * link2.m * link1.l * link2.lc * sin(q2) * q2_dot;
      matC[0][1] = -1.0f * link2.m * link1.l * link2.lc * sin(q2) * q2_dot;
      matC[1][0] = link2.m * link1.l * link2.lc * sin(q2) * q1_dot;
      matC[1][1] = 0.f;

      VecG[0] = GRAVITY * (link1.m * link1.lc * sin(q1) + link2.m * (link1.l * sin(q2) + link2.lc * sin(q1+q2)));
      VecG[1] = GRAVITY * (link2.m * link2.lc * sin(q1+q2));

      VecB[0] = 0;
      VecB[1] = 1;
    }

    float calcErrorState(float qd, float q)
    {
      float dq = fmod(qd, 2*PI) - fmod(q, 2*PI);

      if(dq > PI)
        dq = dq - 2*PI;
      else if (dq <= -PI)
        dq = dq + 2*PI;

      return dq;
    }
  public:
    Acrobot(){}
    ~Acrobot(){}

    void setLinkInform(Link _link1, Link _link2)
    {
        link1 = _link1;
        link2 = _link2;
    }

    void setTargetValue(float *_vecDesQ, float *_vecDesQdot, float *_vecDesQddot)
    {
      int i = 0;
    
      for(i=0; i<2; i++)
      {
        vecDesQ[i] = _vecDesQ[i];
        vecDesQdot[i] = _vecDesQdot[i];
        vecDesQddot[i] = _vecDesQddot[i];      
      }
    }

    float calcControlInput(float *_vecQ, float *_vecQdot)
    {
      float fControlInput = 0;
      fControlInput = doSwingUp(_vecQ, _vecQdot, vecDesQ, vecDesQdot, vecDesQddot);
    }
  
    void setParameter4SwingUp(float fAlpha, float fkp, float fkd)
    {
      alpha = fAlpha;
      kp = fkp;
      kd = fkd;
    }
      
    float doSwingUp(float *vecQ, float *vecQdot, float *vecDesQ, float *vecDesQdot, float *vecDesQddot)
    {
      calcManipulatorDynamics(vecQ, vecQdot);
  
      float H22_bar = matH[1][1] - matH[1][0] / matH[0][0] * matH[1][0];
      if (matH[0][0] == 0) { H22_bar = 0; }
  
      float C2_bar = vecC[1] - matH[1][1] / matH[0][0] * vecC[0];
      if (matH[0][0] == 0 || vecC[0] == 0) { C2_bar = 0; }
  
      float des_q2 = 2 * alpha / PI * atan(vecQdot[0]);
              
      float v2 = vecDesQddot[1] + kd * (vecDesQdot[1] - vecQdot[1]) + kp* calcErrorState(des_q2, vecQ[1]);
  
      float u = H22_bar*v2 + C2_bar;
  
      static int td = 0;
      
      if(td==100)
      {
        td = 0;
      }
      td++;
      
      return u;  
    }
};

