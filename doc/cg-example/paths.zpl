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

set P := { read "P.txt" as "<1n>" skip 2};

param xp[P*A] := read "xp.txt" as "<1n,2n,3n> 4n" skip 2 default 0;

var x[A] binary;
var l[P] >= 0;
var tt >= 0 <= 14;

minimize cost: sum <p> in P: sum <i,j> in A: c[i,j] * xp[p,i,j] * l[p];

subto timeLimit: sum <p> in P: sum <i,j> in A: t[i,j] * xp[p,i,j] * l[p] == tt;

subto pathCover: sum <p> in P: l[p] == 1;

subto linkVars:
   forall <i,j> in A:
      sum <p> in P: xp[p,i,j] * l[p] == x[i,j];


