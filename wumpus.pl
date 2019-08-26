% -*- Mode: Prolog -*-

%% File    : wumpus.pl
%% Author  : Lachlan Clohesy 586856
%% Origin  : 24/05/2018
%% Purpose : Planning software to send robots to shoot a wumpus for 
%%          COMP90048 project4. 

:- module(wumpus,[initialState/5, guess/3, updateState/4]).

%% Takes the dimensions of the input map and the starting position and returns 
%% a State with initialised lists and a list of all coordinates as a Map.
initialState(NR, NC, XS, YS, State0) :-
    allCoords(NR, NC, Map),
    State0 = ((NR,NC),(XS,YS),[],[],[],[], Map, []).
    % State0 needs to store:
    % - (NR, NC) - bounds of the map
    % - (XS, YS) is the start Position
    % - [visited] - list of visited coordinates
    % - [guesses] - list of all previous guesses
    % - [avoid] - list of walls and pits
    % - [suspect] - list of coords that could be where the wumpus is
    % - [map] - list of every coordinate in the map
    % - [Shots] - list of tuples with location and direction of previous shots

%% Takes the initial state and returns a new guess to shoot the wumpus.
guess(State0, State0, Guess) :-
    (_Bounds, Start, Visited, Guesses, Avoid, Suspects, Map, Shots) = State0,
    length(Suspects, L),
    ( L =:= 0 -> % No suspected wumpus location, explore everything
      myDelete(Map, Start, T0), % remove start from targets
      makeAvail(T0, Visited, T1), % remove visited from targets
      makeAvail(T1, Avoid, Target), % remove avoid tiles from target
      % get just the first solution
      once(exploreAll(Start, Map, Avoid, Target, Shots, Guesses, Guess))
    ; L > 0 ->
      % target the suspected wumpus positions rather than unexplored
      once(exploreAll(Start, Map, Avoid, Suspects, Shots, Guesses, Guess))
    ).

updateState(State0, Guess, Feedback, State) :-
    (Bounds, Start, Visited, Guesses, Avoid, Suspects, Map, Shots) = State0,
    readFeedback(Guess, Feedback, Start, (Visited1, Avoid1, Suspects1, Shot1)),
    % update the values for Visited, Avoid and Shot
    myAppend(Visited, Visited1, V0),
    myAppend(Avoid, Avoid1, A0),
    myAppend(Shots, Shot1, Sh0),
    % sort to remove duplicates for increased performance and stability
    sort(V0, V1),
    sort(A0, A1),
    sort(Sh0, Sh1),
    length(Suspects1, L),
    ( L =:= 1 ->
      % location of wumpus must be known
      Sus1 = Suspects1
    ; length(Suspects, L2),
      ( L2 =:= 0 ->
	Sus1 = Suspects1
      ; myIntersection(Suspects, Suspects1, Sus1) 
      % otherwise location must exist in both old and new targets
      )
    ),
    State = (Bounds, Start, V1, [Guess|Guesses], A1, Sus1, Map, Sh1).
    

% Takes a number and returns a list of 1 to that number.
iota(N, Lst) :-
    findall(Num, between(1,N,Num), Lst). 

% Generates all coordinates in a map given number of rows and columns.
allCoords(NR, NC, Lst) :- 
    iota(NR, R),
    iota(NC, C),
    setof((X,Y),(wumpus:myMember(X,R),wumpus:myMember(Y,C)), Lst).

% Gets the direction between two points. Use for neighbouring points.
direction((X1,Y1), (X2,Y2), Dir) :-
    ( Y2 > Y1 ->
      Dir = south
    ; Y1 > Y2 ->
      Dir = north
    ; X1 > X2 ->
      Dir = west
    ; X2 > X1 ->
      Dir = east
    ).

% Check if two points are neighbours.
neighbour(P1, P2) :-
    manhattan(P1, P2, 1).

% Calculate the manhattan distance between two points.
manhattan((X1,Y1), (X2,Y2), Dist) :-
    Dist is abs(X2-X1) + abs(Y2-Y1).

% Takes a point and the map and returns a list of the neighbours to the point.
neighbours(P1, Map, Lst) :-
    findall(Coord, (myMember(Coord, Map), neighbour(P1, Coord)), Lst). 

% Translate the feedback to get places to avoid, target and traversal info
readFeedback(Guess, Feedback, Start, Info) :-
    readFeedback(Guess, Feedback, Start, ([],[],[],[]), Info).
readFeedback(_, [], _Start, Info, Info).
readFeedback([G|Gs], [F|Fs], Start, (V0, A0, Sus0, Shot0), Info) :-
    ( F == miss ->
      readFeedback(Gs, Fs, Start, (V0, A0, Sus0, [(Start,G)|Shot0]), Info)
    ;
    findNext(Start, G, Next),
    ( F == wall ->
      readFeedback(Gs, Fs, Start, (V0, [Next|A0], Sus0, Shot0), Info)
    ; F == pit ->
      readFeedback(Gs, Fs, Start, (V0, [Next|A0], Sus0, Shot0), Info)
    ; F == wumpus ->
      readFeedback(Gs, Fs, Start, (V0, A0, [Next], Shot0), Info)
    ; F == empty ->
      readFeedback(Gs, Fs, Next, ([Next|V0], A0, Sus0, Shot0), Info)
    ; readFeedback(Gs, Fs, Next, ([Next|V0], A0, Sus0, Shot0), Info)
    )
    ).

% Takes a coordinate and a direction and returns the next coordinate.
findNext((A,B), Dir, (C,D)) :-
    ( Dir == north ->
      C is A,
      D is B - 1
    ; Dir == south ->
      C is A,
      D is B + 1
    ; Dir == east ->
      C is A + 1,
      D is B
    ; Dir == west ->
      C is A - 1,
      D is B
    ).

% Explores the map by aiming to move to each target
exploreAll(Start, Map, Avoid, T, Shots, G, Path) :-
    exploreAll(Start, Map, Avoid, Shots, G, start, 0, T, [Start], [], Path).
% Ensure the guess being returned is one that hasn't been made
exploreAll(Start, _Map, _A, _Sho, Gs, _LD, _, [Start|[]], _Prev, Path, Path) :-
    \+ myMember(Path, Gs).
% If the robot reaches a target, move to the next target, reset previous
exploreAll(Start, Map, A, Shots, Gs, LD, Count, [Start|Ts], _Prev,  Ps, Path) :-
    exploreAll(Start, Map, A, Shots, Gs, LD, Count, Ts, [Start],  Ps, Path).
% Ensure energy consumed is at most 100, move to valid locations while heading
% to a target and shoot every 90 degree turn, unless already tried.
exploreAll(Start, Map, A, Shots, Gs, LastDir, Count, [T|Ts], Prev, Ps, Path) :-
    ( Count >= 100 ->
      exploreAll(Start, Map, A, Shots, Gs, LastDir, Count,
		 [Start|[]], Prev, Ps, Path)
    ; neighbours(T, Map, Nbrs),
      ( checkTargetUnavail(Nbrs, A) ->
	exploreAll(Start, Map, A, Shots, Gs, LastDir, Count,
		   Ts, Prev, Ps, Path)
      ;
      makeAvail(Map, A, Avails),
      neighbours(Start, Avails, Options),
      myMember(Next, Options),
      \+ myMember(Next, Prev),
      direction(Start, Next, Dir),
      myDelete(Ts, Start, NewTs),
      NewCount is Count +1,
      myAppend(Ps, [Dir], CurPath),
      ( Dir \== LastDir,
	\+ rev(Dir, LastDir), % stop shooting every time it backtracks
	\+ myMember((Next, Dir), Shots),
	NewCount < 95 ->
	NC is NewCount + 5,
	myAppend(CurPath, [shoot], ShotPath),
	exploreAll(Next, Map, A, [(Next,Dir)|Shots], Gs, Dir, NC,
		   [T|NewTs], [Next|Prev], ShotPath, Path)
      ; exploreAll(Next, Map, A, Shots, Gs, Dir, NewCount,
		   [T|NewTs], [Next|Prev], CurPath, Path)
      )
      )
    ).

% Returns true if all neighbours of a target are unavailable
checkTargetUnavail([],_).
checkTargetUnavail([N|Nbrs], Avoid) :-
    myMember(N, Avoid),
    checkTargetUnavail(Nbrs, Avoid).
    

% Takes two lists and returns a list of all values not shared
makeAvail(Target, [], Target). 
makeAvail(Map, [A|As], Target) :-
    myDelete(Map, A, M1),
    makeAvail(M1, As, Target).

% Check if Dir2 is the opposite direction to Dir1
rev(Dir1, Dir2) :-
    ( Dir1 == north ->
      Dir2 = south
    ; Dir1 == south ->
      Dir2 = north
    ; Dir1 == east ->
      Dir2 = west
    ; Dir1 == west ->
      Dir2 = east
    ).

% Finds the potential locations of the wumpus when first smelled. 
% Omitted from the solution, works but overall performance worsens.
smellOptions(P1, Dir, Map, Opts) :-
    findall(Coord, (myMember(Coord, Map), manhattan(P1, Coord, 3)), All),
    rev(Dir, Opp),
    findNext(P1, Opp, Prev),
    findall(Not, (myMember(Not, Map), manhattan(Prev, Not, 2)), Nots),
    setof(Rem, (wumpus:myMember(Rem, All), \+wumpus:myMember(Rem, Nots)), Opts).

%% My versions of list library files to bypass compile errors.
% Checks if an element is a member of a list.
myMember(Elem, [Elem|_]).
myMember(Elem, [_|Ls]) :-
    myMember(Elem, Ls).

% Takes a list and an element and returns a list without the element.
myDelete([],_,[]).
myDelete([E|As],E,Lst) :-
    myDelete(As,E,Lst).
myDelete([A|As],E,[A|Lst]) :-
    \+ A=E,
    myDelete(As,E,Lst).

% Takes two lists and returns a list concatenating the two.
myAppend([], X, X).
myAppend([X|Y],Z,[X|W]) :-
    myAppend(Y,Z,W).

% Takes two lists and returns a list of the common elements
myIntersection([], _, []).
myIntersection([Head|L1tail], L2, L3) :-
    myMember(Head, L2),
    !, % Stop after first solution
    L3 = [Head|L3tail],
    myIntersection(L1tail, L2, L3tail).
myIntersection([_|L1tail], L2, L3) :-
    myIntersection(L1tail, L2, L3).

