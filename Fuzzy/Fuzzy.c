/*  General-purpose fuzzy inference engine supporting any number of system
inputs and outputs, membership functions, and rules. Membership functions can
be any shape defineable by 2 points and 2 slopes--trapezoids, triangles,
rectanlges, etc. Rules can have any number of antecedents and outputs, and can
vary from rule to rule. "Min" method is used to compute rule strength, "Max"
for applying rule strengths, "Center-of-Gravity" for defuzzification. This
implementation of Inverted Pendulum control problem has: System Inputs, 2
(pendulum angle and velocity); System Outputs, 1 (force supplied to base of
pendulum); Membership Functions, 7 per system input/output; Rules, 15 (each
with 2 antecedents & 1 output). If more precision is required, integers can
be changed to real numbers.*/

#include <stdio.h>
#define MAXNAME 10          /* max number of characters in names           */
#define UPPER_LIMIT  255    /* max number assigned as degree of membership */

/* io_type structure builds a list of system inputs and a list of system
outputs. After initialization, these lists are fixed, except for value field
which is updated on every inference pass. */
struct io_type{
  char name[MAXNAME];        /*  name of system input/output       */
  int value;                 /*  value of system input/output      */
  struct mf_type             /*  list of membership functions for  */
    *membership_functions;   /*     this system input/output       */
  struct io_type *next;      /*  pointer to next input/output      */
  };
/* Membership functions are associated with each system input and output. */
struct mf_type{
  char name[MAXNAME]; /* name of membership function (fuzzy set)    */
  int value;          /* degree of membership or output strength    */
  int point1;         /* leftmost x-axis point of mem. function     */
  int point2;         /* rightmost x-axis point of mem. function    */
  int slope1;         /* slope of left side of membership function  */
  int slope2;         /* slope of right side of membership function */
  struct mf_type *next;   /* pointer to next membership function    */
  };
/*  Each rule has an if side and a then side. Elements making up if side are
pointers to antecedent values inside mf_type structure. Elements making up then
side of rule are pointers to output strength values, also inside mf_type
structure. Each rule structure contains a pointer to next rule in rule base. */
struct rule_element_type{
  int *value;                /* pointer to antecedent/output strength value */
  struct rule_element_type *next; /* next antecedent/output element in rule */
  };
struct rule_type{
  struct rule_element_type *if_side;     /* list of antecedents in rule */
  struct rule_element_type *then_side;   /* list of outputs in rule     */
  struct rule_type *next;                /* next rule in rule base      */
  };
struct rule_type *Rule_Base;             /* list of all rules in rule base */

main()
{
 initialize_system();
 while(1){
  get_system_inputs();
  fuzzification();
  rule_evaluation();
  defuzzification();
  put_system_outputs();
  }
}

/* Fuzzification--Degree of membership value is calculated for each membership
function of each system input. Values correspond to antecedents in rules. */
fuzzification()
{
 struct io_type *si;    /* system input pointer        */
 struct mf_type *mf;    /* membership function pointer */
for(si=System_Inputs; si != NULL; si=si->next)
  for(mf=si->membership_functions; mf != NULL; mf=mf->next)
    compute_degree_of_membership(mf,si->value);
}
/* Rule Evaluation--Each rule consists of a list of pointers to antecedents
(if side), list of pointers to outputs (then side), and pointer to next rule
in rule base. When a rule is evaluated, its antecedents are ANDed together,
using a minimum function, to form strength of rule. Then strength is applied
to each of listed rule outputs. If an output has already been assigned a rule
strength, during current inference pass, a maximum function is used to
determine which strength should apply. */
rule_evaluation()
{
 struct rule_type *rule;
 struct rule_element_type *ip;       /* pointer of antecedents  (if-parts)   */
 struct rule_element_type *tp;       /* pointer to consequences (then-parts) */
 int strength;                /* strength of  rule currently being evaluated */
 for(rule=Rule_Base; rule != NULL; rule=rule->next){
  strength = UPPER_LIMIT;                       /* max rule strength allowed */
        /* process if-side of rule to determine strength */
  for(ip=rule->if_side; ip != NULL; ip=ip->next)
      strength = min(strength,*(ip->value));
       /* process then-side of rule to apply strength */
  for(tp=rule->then_side; tp != NULL; tp=tp->next)
      *(tp->value) = max(strength,*(tp->value));
  }
}
/* Defuzzification */
defuzzification()
{
 struct io_type *so;    /* system output pointer                  */
 struct mf_type *mf;    /* output membership function pointer     */
 int sum_of_products;   /* sum of products of area & centroid */
 int sum_of_areas;     /* sum of shortend trapezoid area          */
 int area;
 int centroid;
 /* compute a defuzzified value for each system output */
for(so=System_Outputs; so != NULL; so=so->next){
  sum_of_products = 0;
  sum_of_areas = 0;
  for(mf=so->membership_functions; mf != NULL; mf=mf->next){
     area = compute_area_of_trapezoid(mf);
     centroid = mf->point1 + (mf->point2 - mf->point1)/2;
     sum_of_products += area * centroid;
     sum_of_areas += area;
     }
  so->value = sum_of_products/sum_of_areas;   /* weighted average */
  }
}

/* Compute Degree of Membership--Degree to which input is a member of mf is
calculated as follows: 1. Compute delta terms to determine if input is inside
or outside membership function. 2. If outside, then degree of membership is 0.
Otherwise, smaller of delta_1 * slope1 and delta_2 * slope2 applies.
3. Enforce upper limit. */
compute_degree_of_membership(mf,input)
struct mf_type *mf;
int input;
{
 int delta_1;
 int delta_2;
 delta_1 = input - mf->point1;
 delta_2 = mf->point2 - input;
 if ((delta_1 <= 0) || (delta_2 <= 0))   /* input outside mem. function ?  */
   mf->value = 0;                           /* then degree of membership is 0 */
 else
   mf->value = min( (mf->slope1*delta_1),(mf->slope2*delta_2) );
   mf->value = min(mf->value,UPPER_LIMIT);  /* enforce upper limit */
}
/* Compute Area of Trapezoid--Each inference pass produces a new set of output
strengths which affect the areas of trapezoidal membership functions used in
center-of-gravity defuzzification. Area values must be recalculated with each
pass. Area of trapezoid is h*(a+b)/2 where h=height=output_strength=mf->value
b=base=mf->point2-mf->point1 a=top= must be derived from h,b, and slopes1&2 */
compute_area_of_trapezoid(mf)
struct mf_type *mf;
{
 int run_1;
 int run_2;
 int base;
 int top;
 int area;
 base = mf->point2 - mf->point1;
 run_1 = mf->value/mf->slope1;
 run_2 = mf->value/mf->slope2;
 top = base - run_1 - run_2;
 area = mf->value * ( base + top)/2;
 return(area);
}
