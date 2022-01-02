#!/usr/bin/env zimpl

set V := { 1 to 6 };

set A := { 
   <1,2>, 
   <1,3>,
   <2,4>,
   <2,5>,
   <3,2>,
   <3,4>,
   <3,5>,
   <4,5>,
   <4,6>,
   <5,6>
};

param c[A] :=
   <1,2> 1, 
   <1,3> 10,
   <2,4> 1,
   <2,5> 2,
   <3,2> 1,
   <3,4> 5,
   <3,5> 12,
   <4,5> 10,
   <4,6> 1,
   <5,6> 2
;

param t[A] :=
   <1,2> 10, 
   <1,3> 3,
   <2,4> 1,
   <2,5> 3,
   <3,2> 2,
   <3,4> 7,
   <3,5> 3,
   <4,5> 1,
   <4,6> 7,
   <5,6> 2
;

var x[A] binary;
var tt >= 0 <= 14;

minimize cost: sum <i,j> in A: c[i,j] * x[i,j];

subto flowSrc: sum <1,j> in A: x[1,j] == 1;

subto flowConserv: 
   forall <i> in { 2 to 5 }:
      sum <j,i> in A: x[j,i] - sum <i,j> in A: x[i,j] == 0;

subto flowSink: sum <i,6> in A: x[i,6] == 1;

subto timeLimit: sum <i,j> in A: t[i,j] * x[i,j] == tt;

