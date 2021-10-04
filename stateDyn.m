function DGU = stateDyn( DGU , i )
%stateDyn
% Function declaring state dynmics for the microgrid

DGU(i).A = [ -1/(DGU(i).Rij(1)*DGU(i).Ct) ,      1/DGU(i).Ct   , 0 ;
                  -1/DGU(i).Lt         , - DGU(i).Rt/DGU(i).Lt , 0 ;
                     -1                ,          0            , 0 ];
                 
                 
DGU(i).Aii = [          0                ,       1/DGU(i).Ct     , 0 ;
                  -1/DGU(i).Lt         , - DGU(i).Rt/DGU(i).Lt , 0 ;
                     -1                ,          0            , 0 ];

DGU(i).B = [      0      ;
             1/DGU(i).Lt ;
                  0      ];

DGU(i).C = eye(size(DGU(i).A,1));

DGU(i).M = [ -1/DGU(i).Ct , 0 ;
                  0       , 0 ;
                  0       , 1 ];

DGU(i).H = [ 1 0 0 ];

DGU(i).Aij = [ 1/(DGU(i).Rij(1)*DGU(i).Ct) , 0 , 0 ;
                        0                  , 0 , 0 ;
                        0                  , 0 , 0 ];
                   
                    
if numel(DGU(i).Ni) ~= 1
    for j = 2:numel(DGU(i).Ni)
        DGU(i).A(1) =  DGU(i).A(1) - 1/(DGU(i).Rij(j)*DGU(i).Ct);
        AijTemp    = [ 1/(DGU(i).Rij(j)*DGU(i).Ct) , 0 , 0 ;
                        0                          , 0 , 0 ;
                        0                          , 0 , 0 ];
        DGU(i).Aij  = [DGU(i).Aij , AijTemp];
    end
end

DGU(i).Ak = DGU(i).A + DGU(i).B*DGU(i).K;