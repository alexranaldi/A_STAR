
% Test script for A_STAR
% Requires A_STAR and A_STAR_PLOT

map = zeros(5);
map(1,:) = 1;
start = 1;
goal = sub2ind(size(map),1,5);
costs = ones(size(map));
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [21    16    11     6     1])

%%

map = zeros(5);
map(:,1) = 1;
start = 1;
goal = 5;
costs = ones(size(map));
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [5     4     3     2     1])

%%

map = [ ...
    0, 1, 0, 0, 0 ; ...
    0, 1, 1, 0, 0 ; ...
    0, 0, 1, 0, 0 ; ...
    0, 0, 0, 1, 0 ; ...
    1, 1, 1, 1, 0 ; ...
    ];
start = 6;
goal = 5;
costs = ones(size(map));
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [5    10    15    19    13     7     6])

%%

map = [ ...
    0, 1, 1, 1, 1 ; ...
    0, 1, 1, 1, 1 ; ...
    0, 0, 1, 1, 1 ; ...
    0, 0, 0, 1, 1 ; ...
    1, 1, 1, 1, 1 ; ...
    ];
    
start = 6;
goal = 5;
costs = ones(size(map));
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [5    10    15    19    13     7     6])

%%

map = [ ...
    0, 1, 1, 1, 1 ; ...
    1, 1, 1, 1, 1 ; ...
    0, 0, 0, 1, 1 ; ...
    0, 1, 0, 1, 1 ; ...
    1, 1, 1, 1, 1 ; ...
    ];
costs  = [ ...
    1,   1,  1,  1,  1 ; ...
    1,   10, 9,  9,  2 ; ...
    100, 0,  0,  9,  1 ; ...
    1,   1,  0,  1,  1 ; ...
    1,   9,  1,  1,  1 ; ...
    ];
start = 6;
goal = 5;
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [5    9    15    19    23    22    16    11     6])


%%

map = [ ...
    1, 1, 1, 1, 1 ; ...
    1, 0, 0, 0, 1 ; ...
    1, 1, 0, 1, 1 ; ...
    0, 0, 0, 1, 0 ; ...
    1, 1, 1, 1, 1 ; ...
    ];
start = 8;
goal = 5;
costs = ones(size(map));
final = a_star(logical(map), costs, start, goal);
a_star_plot(logical(map), costs, final)
isequal(final, [5    10    15    19    18    22    16    11     6   2   8])

%%

map = false(20);
map(1,1) = true; % start
map(20, 1) = true; % goal
start = 1;
goal = 20;
map(:,1) = true;
map(end,:)=true;
map(4,:)=true;
map(:,end)=true;
map(end-1,:)=true;
map(:,14)=true;
map(5:10,6:10)=true;
costs = ones(size(map));
costs(7,1) = 100;
costs(end, 3)=50;
costs(end, 5)=50;
costs(end, 7)=50;
costs(end-1, 9)=50;
costs(end-1, 10)=50;
costs(end, 11)=50;
costs(5,14)=25;
costs(6,14)=30;
costs(4,8) = 60;
costs(5,8) = 60;
costs(19,4) = 60;
costs(5,7) = 90;
costs(6,7) = 35;
costs([4 5 6],9) = 75;
costs(7,9) = 25;
costs(8,9) = 15;
costs(9,9) = 20;
costs(10,9) = 25;
tic
final = a_star((map), costs, start, goal);
toc
a_star_plot((map), costs, final)
