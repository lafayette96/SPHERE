#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define max(a,b)   (a<b ? b : a)
#define min(a,b)   (a>b ? b : a)
#define MAXNAME 10
#define UPPER_LIMIT 100.0

float  upperLimit = 1;

struct rule_element_type
{
   float *value;
   struct rule_element_type *next;
};

struct rule_type
{
   struct rule_element_type if_side[2];
   struct rule_element_type then_side;
   //struct rule_type *next;
   float gain;
};

struct mf_type
{
   char name[MAXNAME];
   float value;
   float point1;
   float point2;
   float slope1;
   float slope2;
};

struct io_type
{
   char name[MAXNAME];
   float value;
   struct mf_type membership_functions[3];
   struct io_type *next;
};

struct io_type System_Inputs[2];
struct io_type System_Output;
struct rule_type Rule_Base[6];
struct mf_type *mfptr_temp;
struct mf_type *top_mf_temp;
float test_t;


void instert_memberfunction(struct io_type *ioptr, char *buff, float a, float b, float c, float d, int n)
{
    sprintf(ioptr->membership_functions[n].name,"%s", buff);
    ioptr->membership_functions[n].point1 = a;
    ioptr->membership_functions[n].point2 = d;
    if(b-a > 0)
        ioptr->membership_functions[n].slope1 = 1/(b-a);
    else
    {
        return;
    }
    if(d-c > 0)
        ioptr->membership_functions[n].slope2 = 1/(d-c);
    else
    {
        return;
    }
		return;
}


void readFuzzySet(struct io_type *iptr1, struct io_type *iptr2, struct io_type *optr)
{
//   mfptr_temp = NULL;
//   top_mf_temp = NULL;
//   instert_memberfunction(iptr1, "Le",   -61.0,  -60.0, -40.0,   0.0, 0);
//   instert_memberfunction(iptr1, "Ze",   -40.0,    0.0,   0.0,  40.0, 1);
//   instert_memberfunction(iptr1, "He",     0.0,   40.0,  60.0,  61.0, 2);

//   mfptr_temp = NULL;
//   top_mf_temp = NULL;
//   instert_memberfunction(iptr2, "Lde", -121.0, -120.0, -80.0,   0.0 ,0);
//   instert_memberfunction(iptr2, "Zde",  -80.0,    0.0,   0.0,  80.0, 1);
//   instert_memberfunction(iptr2, "Hde",    0.0,   80.0, 120.0, 121.0, 2);

   mfptr_temp = NULL;
   top_mf_temp = NULL;
   instert_memberfunction(optr, "L",   -61.0,  -60.0, -40.0,   0.0,  0);
   instert_memberfunction(optr, "Z",   -40.0,    0.0,   0.0,  40.0,  1);
   instert_memberfunction(optr, "H",     0.0,   40.0,  60.0,  61.0,  2);
	
	 mfptr_temp = NULL;
   top_mf_temp = NULL;
   instert_memberfunction(iptr1, "Le",   -71.0,  -70.0, -50.0,   0.0, 0);
   instert_memberfunction(iptr1, "Ze",   -40.0,    0.0,   0.0,  40.0, 1);
   instert_memberfunction(iptr1, "He",     0.0,   50.0,  70.0,  70.0, 2);

   mfptr_temp = NULL;
   top_mf_temp = NULL;
   instert_memberfunction(iptr2, "Lde", -151.0, -150.0, -90.0,   0.0 ,0);
   instert_memberfunction(iptr2, "Zde",  -80.0,    0.0,   0.0,  80.0, 1);
   instert_memberfunction(iptr2, "Hde",    0.0,   90.0, 150.0, 151.0, 2);

   mfptr_temp = NULL;
   top_mf_temp = NULL;
   instert_memberfunction(optr, "L",   -71.0,  -70.0, -45.0,   0.0,  0);
   instert_memberfunction(optr, "Z",   -40.0,    0.0,   0.0,  40.0,  1);
   instert_memberfunction(optr, "H",     0.0,   45.0,  70.0,  71.0,  2);
	 
//	 mfptr_temp = NULL;
//   top_mf_temp = NULL;
//   instert_memberfunction(iptr1, "Le",   -80.0,  -60.0, -40.0,   0.0, 0);
//   instert_memberfunction(iptr1, "Ze",   -40.0,    0.0,   0.0,  40.0, 1);
//   instert_memberfunction(iptr1, "He",     0.0,   40.0,  60.0,  80.0, 2);

//   mfptr_temp = NULL;
//   top_mf_temp = NULL;
//   instert_memberfunction(iptr2, "Lde", -121.0, -120.0, -80.0,   0.0 ,0);
//   instert_memberfunction(iptr2, "Zde",  -80.0,    0.0,   0.0,  80.0, 1);
//   instert_memberfunction(iptr2, "Hde",    0.0,   80.0, 120.0, 121.0, 2);

//   mfptr_temp = NULL;
//   top_mf_temp = NULL;
//   instert_memberfunction(optr, "L",   -80.0,  -60.0, -40.0,   0.0,  0);
//   instert_memberfunction(optr, "Z",   -40.0,    0.0,   0.0,  40.0,  1);
//   instert_memberfunction(optr, "H",     0.0,   40.0,  60.0,  80.0,  2);
}


void instert_rule(struct rule_type *ruleptr, char *buff, char *buff1, char *buff2, float buff_gain)
{
	   int j = 0;
     
      for(j=0;  j<3; j++)
      {
         if((strcmp(System_Inputs[0].membership_functions[j].name, buff) == 0) || (strcmp("0", buff) == 0))
         {
            if((strcmp("0", buff) == 0))
               ruleptr->if_side[0].value = &upperLimit;
            else
               ruleptr->if_side[0].value = &System_Inputs[0].membership_functions[j].value;
					
            break;
         }
      }
      for(j=0;  j<3; j++)
      {
         if((strcmp(System_Inputs[1].membership_functions[j].name, buff1) == 0) || (strcmp("0", buff1) == 0))
         {
            if((strcmp("0", buff1) == 0))
               ruleptr->if_side[1].value = &upperLimit;
            else
               ruleptr->if_side[1].value = &System_Inputs[1].membership_functions[j].value;
            break;
         }
      }

      for(j=0;  j<3; j++)
      {
         if((strcmp(System_Output.membership_functions[j].name, buff2)) == 0)
         {
            ruleptr->then_side.value = &System_Output.membership_functions[j].value;
            break;
         }
      }
      ruleptr->gain = buff_gain;
}

void initialize_system()
{

   readFuzzySet(&System_Inputs[0], &System_Inputs[1], &System_Output);


   instert_rule(&Rule_Base[0], "Le",  "0",  "H", 0.7);
   //Rule_Base[0].if_side->value = &(System_Inputs->membership_functions->value);
   instert_rule(&Rule_Base[1], "Ze",  "0",  "Z", 0.6);
	 //Rule_Base[1].if_side->value = &(System_Inputs->membership_functions->next->value);
   instert_rule(&Rule_Base[2], "He",  "0",  "L", 0.7);
	 //Rule_Base[2].if_side->value = &(System_Inputs->membership_functions->next->next->value);
   instert_rule(&Rule_Base[3], "0", "Lde",  "L", 0.4);
	 //Rule_Base[3].if_side->next->value = &(System_Inputs->next->membership_functions->value);
   instert_rule(&Rule_Base[4], "0", "Zde",  "Z", 0.15);
	 //Rule_Base[4].if_side->next->value = &(System_Inputs->next->membership_functions->next->value);
   instert_rule(&Rule_Base[5], "0", "Hde",  "H", 0.4);
	 //Rule_Base[4].if_side->next->value = &(System_Inputs->next->membership_functions->next->next->value);

   return;
}

void get_system_inputs(float input1, float input2)
{
   struct io_type *ioptr;
   ioptr = System_Inputs;
   ioptr->value = input1;
   ioptr = ioptr->next;
   ioptr->value = input2;

   return;
}

void rule_evaluation()
{
   int i = 0;
	 int j = 0;

   float strength;
   for(i=0; i<6; i++)
   {
      strength = 1.0;
      for(j=0;j<2;j++)
      {
         strength = min(strength, *(Rule_Base[i].if_side[j].value));
      }

      *(Rule_Base[i].then_side.value) = max(strength*(Rule_Base[i].gain),*(Rule_Base[i].then_side.value));



   }

   return;
}

void compute_degree_of_membership(struct mf_type *mf, float input)
{
   float delta_1, delta_2;
   delta_1 = input - mf->point1;
   delta_2 = mf->point2 - input;

   if((delta_1 <= 0) || (delta_2 <= 0))
      mf->value = 0;
   else
   {
      mf->value = min((mf->slope1*delta_1), (mf->slope2*delta_2));
      mf->value = min(mf->value, 1);
   }
   return;
}

void fuzzification()
{
	 int i = 0;
	 int j = 0;
   for(i=0; i<2; i++)
			for(j=0; j<3; j++)
			{
         compute_degree_of_membership(&System_Inputs[i].membership_functions[j], System_Inputs[i].value);
			}
   return;
}

void fuzzy_flush()
{
   int i =0;
   for(i = 0; i<6; i++)
        *(Rule_Base[i].then_side.value) = 0.0;

   return;
}

float compute_area_of_trapezoid(struct mf_type *mf, float *base, float *top)
{
   float run_1, run_2, area;
   *base = mf->point2 - mf->point1;
   run_1 = mf->value / mf->slope1;
   run_2 = mf->value / mf->slope2;
   *top = *base - run_1 - run_2;
   area = mf->value*(*base+*top)/2;

   return area;
}

void defuzzification()
{
   int i = 0;
	 int j = 0;
   float top;
   float base;
   float sum_of_products;
   float sum_of_areas;
   float area, centroid;

      sum_of_products = 0;
      sum_of_areas = 0;
      for(j=0; j<3; j++, i++)
      {
         area = compute_area_of_trapezoid(&System_Output.membership_functions[j], &base, &top);
         if (i == 1)
            centroid = System_Output.membership_functions[j].point1 + (System_Output.membership_functions[j].point2 - System_Output.membership_functions[j].point1)/2;
         else if(i == 0)
            centroid = -(2*base*base - top*top + 2*base*top)/(3*(base+top));
         else
            centroid = (2*base*base - top*top + 2*base*top)/(3*(base+top));
         sum_of_products += area*centroid;
         sum_of_areas += area;

      }
      if(sum_of_areas == 0)
      {
         System_Output.value = 0;
         return;
      }
      System_Output.value = sum_of_products/sum_of_areas;
			
   return;
}

float fuzzy_controller(float input1, float input2)
{

   fuzzy_flush();
   System_Inputs[0].value = input1;
   System_Inputs[1].value = input2;
   fuzzification();
   rule_evaluation();
   defuzzification();

   return System_Output.value;
}
