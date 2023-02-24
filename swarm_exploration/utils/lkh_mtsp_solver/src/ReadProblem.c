#include "LKH.h"
#include "Heap.h"

/*
 * The ReadProblem function reads the problem data in TSPLIB format from the
 * file specified in the parameter file (PROBLEM_FILE).
 *
 * The following description of the file format is extracted from the TSPLIB
 * documentation.
 *
 * The file consists of a specification part and a data part. The specification
 * part contains information on the file format and on its contents. The data
 * part contains explicit data.
 *
 * (1) The specification part
 *
 * All entries in this section are of the form <keyword> : <value>, where
 * <keyword> denotes an alphanumerical keyword and <value> denotes
 * alphanumerical or numerical data. The terms <string>, <integer> and <real>
 * denote character string, integer or real data, respectively. The order of
 * specification of the keywords in the data file is arbitrary (in principle),
 * but must be consistent, i.e., whenever a keyword is specified, all
 * necessary information for the correct interpretation of the keyword has to
 * be known.
 *
 * Below is given a list of all available keywords.
 *
 * NAME : <string>e
 * Identifies the data file.
 *
 * TYPE : <string>
 * Specifies the type of data. Possible types are
 * TSP          Data for a symmetric traveling salesman problem
 * ATSP         Data for an asymmetric traveling salesman problem
 * SOP          Data for a sequence ordering problem
 * HCP          Hamiltonian cycle problem data
 * HPP          Hamiltonian path problem data (not available in TSPLIB)
 * TSPTW        Data for a TSP instance with time windows
 * CCVRP        Data for a cumulative capacitated vehicle routing problem
 * CVRP         Data for a symmetric capacitated vehicle routing problem
 * ACVRP        Data for an asymmetric capacitated vehicle routing problem
 * CVRPTW       Data for a capacitated vehicle routing problem with
 *              time windows
 * VRPMPD       Data for a mixed pickup and delivery problem with backhauls
 * 1-PDTSP      Data for a one-commodity pickup-and-delivery traveling
 *              salesman problem
 * MLP          Data for a minimum latency problem
 * m-PDTSP      Data for a mulity-commodity pickup-and-delivery traveling
 *              salesman problem
 * m1-PDTSP     Data for a mulity-commodity one-to-one pickup-and-delivery
 *              traveling salesman problem
 * OVRP         Data for an open vehicle routing problem
 * PDTSP        Data for a pickup and delivery traveling salesman problem
 * PDTSPF       Data for a pickup and delivery traveling salesman problem
 *              with FIFO loading
 * PDTSPL       Data for a pickup and delivery traveling salesman problem
 *              with LIFO loading
 * PDPTW        Data for a pickup and delivery problem with time windows
 * RCTVRP       Data for a risk-constrained cash-in-transit vehicle
 *              routing problem
 * RCTVRPTW     Data for a risk-constrained cash-in-transit vehicle
 *              routing problem with time windows
 * TRP          Data for a traveling repairman problem
 * TSPDL        Dara for a traveling salesman problem with draft limits
 * TSPPD        Data for a pickup and delivery travling salesman problem
 * TSTSP        Data for a Steiner traveling salesman problem
 * VRPB         Data for a vehicle routing problem with backhauls
 * VRPBTW       Data for a vehicle routing problem with backhauls and
 *              time windows
 * CTSP         Data for a colored traveling salesman problem
 *
 * COMMENT : <string>
 * Additional comments (usually the name of the contributor or the creator of
 * the problem instance is given here).
 *
 * DIMENSION : < integer>
 * The number of nodes.
 *
 * CAPACITY : <integer>
 * Specifies the truck capacity in a CVRP.
 *
 * DISTANCE : <real>
 * The maximum length allowed for each route in a CVRP.
 *
 * EDGE_WEIGHT_TYPE : <string>
 * Specifies how the edge weights (or distances) are given. The values are:
 * ATT          Special distance function for problem att48 and att532
 * CEIL_2D      Weights are Euclidean distances in 2-D rounded up
 * CEIL_3D      Weights are Euclidean distances in 3-D rounded up
 * EUC_2D       Weights are Euclidean distances in 2-D
 * EUC_3D       Weights are Euclidean distances in 3-D
 * EXACT_2D     Weights are EUC_2D distances (SCALE = 1000 as default)
 * EXACT_3D     Weights are EUC_3D distances (SCALE = 1000 as default)
 * EXPLICIT     Weights are listed explicitly in the corresponding section
 * FLOOR_2D     Weights are Euclidean distances in 2-D rounded down
 * FLOOR_3D     Weights are Euclidean distances in 3-D rounded down
 * GEO          Weights are geographical distances in kilometers (TSPLIB)
 *              Coordinates are given in the form DDD.MM where DDD are the
 *              degrees and MM the minutes
 * GEOM         Weights are geographical distances in meters (used for the
 *              world TSP). Coordinates are given in decimal form
 * GEO_MEEUS    Weights are geographical distances in kilometers, computed
 *              according to Meeus' formula.  Coordinates are given in the
 *              form DDD.MM where DDD are the degrees and MM the minutes
 * GEOM_MEEUS   Weights are geographical distances, computed according to
 *              Meeus' formula. Coordinates are given in decimal form
 * MAN_2D       Weights are Manhattan distances in 2-D
 * MAN_3D       Weights are Manhattan distances in 3-D
 * MAX_2D       Weights are maximum distances in 2-D
 * MAX_3D       Weights are maximum distances in 3-D
 * TOR_2D       Wirghes are toroidal distances in 2-D
 * TOR_3D       Wirghes are toroidal distances in 3-D
 * XRAY1        Distance function for crystallography problems (Version 1)
 * XRAY2        Distance function for crystallography problems (Version 2)
 * SPECIAL      There is a special distance function implemented in
 *              the Distance_SPECIAL function.
 *
 * EDGE-WEIGHT_FORMAT : <string>
 * Describes the format of the edge weights if they are given explicitly.
 * The values are
 * FUNCTION         Weights are given by a function (see above)
 * FULL_MATRIX      Weights are given by a full matrix
 * UPPER_ROW        Upper triangular matrix
 *                      (row-wise without diagonal entries)
 * LOWER_ROW        Lower triangular matrix
 *                      (row-wise without diagonal entries)
 * UPPER_DIAG_ROW   Upper triangular matrix
 *                      (row-wise including diagonal entries)
 * LOWER_DIAG_ROW   Lower triangular matrix
 *                      (row-wise including diagonal entries)
 * UPPER_COL        Upper triangular matrix
 *                      (column-wise without diagonal entries)
 * LOWER_COL        Lower triangular matrix
 *                      (column-wise without diagonal entries)
 * UPPER_DIAG_COL   Upper triangular matrix
 *                      (column-wise including diagonal entries)
 * LOWER_DIAG_COL   Lower triangular matrix
 *                      (column-wise including diagonal entries)
 *
 * EDGE_DATA_FORMAT : <string>
 * Describes the format in which the edges of a graph are given, if the
 * graph is not complete. The values are
 * EDGE_LIST    The graph is given by an edge list
 * ADJ_LIST     The graph is given by an adjacency list
 *
 * NODE_COORD_TYPE : <string>
 * Specifies whether the coordinates are associated with each node
 * (which, for example may be used for either graphical display or
 * distance computations.
 * The values are
 * TWOD_COORDS      Nodes are specified by coordinates in 2-D
 * THREED_COORDS    Nodes are specified by coordinates in 3-D
 * NO_COORDS        The nodes do not have associated coordinates
 * The default value is NO_COORDS. In the current implementation, however,
 * the value has no significance.
 *
 * DISPLAY_DATA_TYPE : <string>
 * Specifies how a graphical display of the nodes can be obtained.
 * The values are
 * COORD_DISPLAY    Display is generated from the node coordinates
 * TWOD_DISPLAY     Explicit coordinates in 2-D are given
 * NO_DISPLAY       No graphical display is possible
 *
 * The default value is COORD_DISPLAY if node coordinates are specifies and
 * NO_DISPLAY otherwise. In the current implementation, however, the value
 * has no significance.
 *
 * DEMAND_DIMENSION : < integer>
 * The number of objects in an m1-PDTSP.
 *
 * GRID_SIZE : <real>
 * The grid size for toroidal instances.
 * Default: 1000000.0
 *
 * RISK_THRESHOLD : <integer>
 * The maximum risk alllowed for each route in an RCTVRP or RCTVRPTW instance.
 *
 * SALESMEN : <integer>
 * VEHICLES : <integer>
 * The number of vehicles/salesmen in a CVRP.
 *
 * SCALE : <integer>
 * Scale factor. Distances are multiplied by this factor.
 *
 * EOF
 * Terminates input data. The entry is optional.
 *
 * (2) The data part
 *
 * Depending on the choice of specifications some additional data may be
 * required. These data are given corresponding data sections following the
 * specification part. Each data section begins with the corresponding
 * keyword. The length of the sectionis either explicitly known form the
 * format specification, or the section is terminated by an appropriate
 * end-of-section identifier.
 *
 * NODE_COORD_SECTION :
 * Node coordinates are given in this section. Each line is of the form
 *
 *      <integer> <real> <real>
 *
 * if NODE_COORD_TYPE is TWOD_COORDS, or
 *
 *      <integer> <real> <real> <real>
 *
 * if NODE_COORD_TYPE is THREED_COORDS. The integers give the number of the
 * respective nodes. The real numbers are the associated coordinates.
 *
 * EDGE_DATA_SECTION :
 * Edges of the graph are specified in either of the two formats allowed in
 * the EDGE_DATA_FORMAT entry. If the type is EDGE_LIST, then the edges are
 * given as a sequence of lines of one of the forms
 *
 *      <integer> <integer>
 *      <integer> <integer> <integer>
 *
 * each entry giving the terminal nodes of some edge, and if three integers are
 * given, the last one specifies its weight. The list is terminated by a -1.
 * If the type is ADJ_LIST, the section consists of adjacency lists for nodes.
 * The adjacency list of a node x is specified as
 *
 *      <integer> <integer> ... <integer> -1
 *
 * where the first integer gives the number of node x and the following
 * integers (terminated by -1) the numbers of the nodes adjacent to x.
 * The list of adjacency lists are terminated by an additional -1.
 *
 * FIXED_EDGES_SECTION :
 * In this section, edges are listed that are required to appear in each
 * solution to the problem. The edges to be fixed are given in the form
 * (per line)
 *
 *      <integer> <integer>
 *
 * meaning that the edge (arc) from the first node to the second node has
 * to be contained in a solution. This section is terminated by a -1.
 *
 * DISPLAY_DATA_SECTION :
 * If DISPLAY_DATA_TYPE is TWOD_DISPLAY, the 2-dimensional coordinates from
 * which a display can be generated are given in the form (per line)
 *
 *      <integer> <real> <real>
 *
 * The integers specify the respective nodes and the real numbers give the
 * associated coordinates. The contents of this section, however, has no
 * significance in the current implementation.
 *
 * EDGE_WEIGHT_SECTION :
 * The edge weights are given in the format specifies by the EDGE_WEIGHT_FORMAT
 * entry. At present, all explicit data are integral and is given in one of the
 * (self-explanatory) matrix formats, with explicitly known lengths.
 *
 * TOUR_SECTION :
 * A tour is specified in this section. The tour is given by a list of
 * integers giving the sequence in which the nodes are visited in the tour.
 * The tour is terminated by a -1. Note: In contrast to the TSPLIB format,
 * only one tour can be given in this section. The tour is used to limit
 * the search (the last edge to be excluded in a non-gainful move must not
 * belong to the tour). In addition, the Alpha field of its edges is set to
 * -1.
 *
 * BACKHAUL_SECTION :
 * This section is used for specifying VRPB instances.
 * It contains a list of backhaul nodes. This list is terminated by a -1.
 *
 * CTSP_SET_SECTION :
 * This section is used for specifying CTSP instances.
 * Each entry has the following format:
 * c v1 v2 ... vk -1, where c is the color number (colors are numbered
 * from 1 to SALESMEN), and v1 v2 ... vk are vertices with color c
 * (vertices are numbered from 1 to Dimension).
 *
 * The first integer gives the number of the node. The last two integers
 * give the earliest and latest time for the node.
 *
 * DEMAND_SECTION :
 * The demands of all nodes of a CVRP are given in the form (per line)
 *
 *    <integer> <integer>
 *
 * The first integer spcifies a node number, the second its demand. The depot
 * nodes must also occcur in this section. Their demands are 0.
 *
 * DEPOT_SECTION :
 * Contains a list of possible alternate depot nodes. This list is terminated
 * by a -1. The current implementation allows only one depot.
 *
 * DRAFT_LIMIT_SECTION :
 * The draft limits of all nodes of a CVRP are give in the form (per line)
 *
 *    <integer> <integer>
 *
 * The first integer spcifies a node number, the second its draft limit.
 * The depot nodes must also occcur in this section. Their demands are 0.
 *
 * PICKUP_AND_DELIVERY_SECTION :
 * This section is used for specifying specifying pickup-and-delivery
 * instances. Each line is of the form
 *
 *     <integer> <integer> <real> <real> <real> <integer> <integer>
 *
 * The first integer gives the number of the node.
 * The second integer gives its demand (ignored for PDTSPF, PDTSPL, VRPMPD
 * and VRPSPD instances).
 * The third and fourth number give the earliest and latest time for the node.
 * The fifth number specifies the service time for the node.
 * The last two integers are used to specify pickup and delivery. For a PDPTW,
 * PDTSP, PDTSPF and PDTSPL instance, the first of these integers gives the
 * index of the pickup sibling, whereas the second integer gives the index of
 * the delivery sibling. For a VRPMPD and VRPSPD instance, the two integers
 * simply give the size of the pickup and delivery for the node.
 *
 * REIQUIRED_NODES_SECTION :
 * Contains a list of required nodes for a Steiner traveling salesman problem.
 * This list is terminated * by a -1.
 *
 * SERVICE_TIME_SECTION :
 * The service times of all nodes of a CVRP are given in the form (per line)
 *
 *    <integer> <real>
 *
 * The integer specifies a node number, the real its service time.
 * The depot node must also occur in this section. Its service time is 0.
 *
 * TIME_WINDOW_SECTION :
 * Time windows are given in this section. Each line is of the form
 *
 *      <integer> <real> <realr>
 *
 * The first integer specifies a node number. The two reals specify
 * earliest and latest arrival time for the node, respectively.
 */

static const char Delimiters[] = " :=\n\t\r\f\v\xef\xbb\xbf";
static void CheckSpecificationPart(void);
static char *Copy(char *S);
static void CreateNodes(void);
static int FixEdge(Node * Na, Node * Nb);
static void Read_BACKHAUL_SECTION(void);
static void Read_CAPACITY(void);
static void Read_CTSP_SET_SECTION(void);
static void Read_DEMAND_DIMENSION(void);
static void Read_DEMAND_SECTION(void);
static void Read_DEPOT_SECTION(void);
static void Read_DIMENSION(void);
static void Read_DISPLAY_DATA_SECTION(void);
static void Read_DISPLAY_DATA_TYPE(void);
static void Read_DISTANCE(void);
static void Read_DRAFT_LIMIT_SECTION(void);
static void Read_EDGE_DATA_FORMAT(void);
static void Read_EDGE_DATA_SECTION(void);
static void Read_EDGE_WEIGHT_FORMAT(void);
static void Read_EDGE_WEIGHT_SECTION(void);
static void Read_EDGE_WEIGHT_TYPE(void);
static void Read_FIXED_EDGES_SECTION(void);
static void Read_GRID_SIZE(void);
static void Read_NAME(void);
static void Read_NODE_COORD_SECTION(void);
static void Read_NODE_COORD_TYPE(void);
static void Read_PICKUP_AND_DELIVERY_SECTION(void);
static void Read_REQUIRED_NODES_SECTION(void);
static void Read_RISK_THRESHOLD(void);
static void Read_SALESMEN(void);
static void Read_SCALE(void);
static void Read_SERVICE_TIME(void);
static void Read_SERVICE_TIME_SECTION(void);
static void Read_TIME_WINDOW_SECTION(void);
static void Read_TOUR_SECTION(FILE ** File);
static void Read_TYPE(void);
static int TwoDWeightType(void);
static int ThreeDWeightType(void);
static void Convert2FullMatrix();

void ReadProblem()
{
    int i, j, K;
    char *Line, *Keyword;

    if (!(ProblemFile = fopen(ProblemFileName, "r")))
        eprintf("Cannot open PROBLEM_FILE: \"%s\"", ProblemFileName);
    if (TraceLevel >= 1)
        printff("Reading PROBLEM_FILE: \"%s\" ... ", ProblemFileName);
    FreeStructures();
    FirstNode = 0;
    WeightType = WeightFormat = ProblemType = -1;
    CoordType = NO_COORDS;
    Name = Copy("Unnamed");
    Type = EdgeWeightType = EdgeWeightFormat = 0;
    EdgeDataFormat = NodeCoordType = DisplayDataType = 0;
    Distance = 0;
    C = 0;
    c = 0;
    DistanceLimit = DBL_MAX;
    while ((Line = ReadLine(ProblemFile))) {
        if (!(Keyword = strtok(Line, Delimiters)))
            continue;
        for (i = 0; i < (int) strlen(Keyword); i++)
            Keyword[i] = (char) toupper(Keyword[i]);
        if (!strcmp(Keyword, "COMMENT"));
        else if (!strcmp(Keyword, "BACKHAUL_SECTION"))
            Read_BACKHAUL_SECTION();
        else if (!strcmp(Keyword, "CAPACITY"))
            Read_CAPACITY();
        else if (!strcmp(Keyword, "CTSP_SET_SECTION"))
            Read_CTSP_SET_SECTION();
        else if (!strcmp(Keyword, "DEMAND_DIMENSION"))
            Read_DEMAND_DIMENSION();
        else if (!strcmp(Keyword, "DEMAND_SECTION"))
            Read_DEMAND_SECTION();
        else if (!strcmp(Keyword, "DEPOT_SECTION"))
            Read_DEPOT_SECTION();
        else if (!strcmp(Keyword, "DIMENSION"))
            Read_DIMENSION();
        else if (!strcmp(Keyword, "DISPLAY_DATA_SECTION"))
            Read_DISPLAY_DATA_SECTION();
        else if (!strcmp(Keyword, "DISPLAY_DATA_TYPE"))
            Read_DISPLAY_DATA_TYPE();
        else if (!strcmp(Keyword, "DISTANCE"))
            Read_DISTANCE();
        else if (!strcmp(Keyword, "DRAFT_LIMIT_SECTION"))
            Read_DRAFT_LIMIT_SECTION();
        else if (!strcmp(Keyword, "EDGE_DATA_FORMAT"))
            Read_EDGE_DATA_FORMAT();
        else if (!strcmp(Keyword, "EDGE_DATA_SECTION"))
            Read_EDGE_DATA_SECTION();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_FORMAT"))
            Read_EDGE_WEIGHT_FORMAT();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_SECTION"))
            Read_EDGE_WEIGHT_SECTION();
        else if (!strcmp(Keyword, "EDGE_WEIGHT_TYPE"))
            Read_EDGE_WEIGHT_TYPE();
        else if (!strcmp(Keyword, "EOF"))
            break;
        else if (!strcmp(Keyword, "FIXED_EDGES_SECTION"))
            Read_FIXED_EDGES_SECTION();
        else if (!strcmp(Keyword, "GRID_SIZE"))
            Read_GRID_SIZE();
        else if (!strcmp(Keyword, "NAME"))
            Read_NAME();
        else if (!strcmp(Keyword, "NODE_COORD_SECTION"))
            Read_NODE_COORD_SECTION();
        else if (!strcmp(Keyword, "NODE_COORD_TYPE"))
            Read_NODE_COORD_TYPE();
        else if (!strcmp(Keyword, "PICKUP_AND_DELIVERY_SECTION"))
            Read_PICKUP_AND_DELIVERY_SECTION();
        else if (!strcmp(Keyword, "REQUIRED_NODES_SECTION"))
            Read_REQUIRED_NODES_SECTION();
        else if (!strcmp(Keyword, "RISK_THRESHOLD"))
            Read_RISK_THRESHOLD();
        else if (!strcmp(Keyword, "SALESMEN") ||
                 !strcmp(Keyword, "VEHICLES"))
            Read_SALESMEN();
        else if (!strcmp(Keyword, "SCALE"))
            Read_SCALE();
        else if (!strcmp(Keyword, "SERVICE_TIME"))
            Read_SERVICE_TIME();
        else if (!strcmp(Keyword, "SERVICE_TIME_SECTION"))
            Read_SERVICE_TIME_SECTION();
        else if (!strcmp(Keyword, "TIME_WINDOW_SECTION"))
            Read_TIME_WINDOW_SECTION();
        else if (!strcmp(Keyword, "TOUR_SECTION"))
            Read_TOUR_SECTION(&ProblemFile);
        else if (!strcmp(Keyword, "TYPE"))
            Read_TYPE();
        else
            eprintf("Unknown keyword: %s", Keyword);
    }
    Swaps = 0;

    /* Adjust parameters */
    if (Seed == 0)
        Seed = (unsigned) time(0);
    if (Precision == 0)
        Precision = 100;
    if (InitialStepSize == 0)
        InitialStepSize = 1;
    if (MaxSwaps < 0)
        MaxSwaps = Dimension;
    if (KickType > Dimension / 2)
        KickType = Dimension / 2;
    if (Runs == 0)
        Runs = 10;
    if (MaxCandidates > Dimension - 1)
        MaxCandidates = Dimension - 1;
    if (ExtraCandidates > Dimension - 1)
        ExtraCandidates = Dimension - 1;
    if (Scale < 1)
        Scale = 1;
    if (SubproblemSize >= Dimension)
        SubproblemSize = Dimension;
    else if (SubproblemSize == 0) {
        if (AscentCandidates > Dimension - 1)
            AscentCandidates = Dimension - 1;
        if (InitialPeriod < 0) {
            InitialPeriod = Dimension / 2;
            if (InitialPeriod < 100)
                InitialPeriod = 100;
        }
        if (Excess < 0)
            Excess = 1.0 / DimensionSaved * Salesmen;
        if (MaxTrials == -1)
            MaxTrials = Dimension;
        HeapMake(Dimension);
    }
    if (POPMUSIC_MaxNeighbors > Dimension - 1)
        POPMUSIC_MaxNeighbors = Dimension - 1;
    if (POPMUSIC_SampleSize > Dimension)
        POPMUSIC_SampleSize = Dimension;
    Depot = &NodeSet[MTSPDepot];
    if (ProblemType == CVRP) {
        Node *N;
        int MinSalesmen;
        if (Capacity <= 0)
            eprintf("CAPACITY not specified");
        TotalDemand = 0;
        N = FirstNode;
        do
            TotalDemand += N->Demand;
        while ((N = N->Suc) != FirstNode);
        MinSalesmen =
            TotalDemand / Capacity + (TotalDemand % Capacity != 0);
        if (Salesmen == 1) {
            Salesmen = MinSalesmen;
            if (Salesmen > Dimension)
                eprintf("CVRP: SALESMEN larger than DIMENSION");
        } else if (Salesmen < MinSalesmen)
            eprintf("CVRP: SALESMEN too small to meet demand");
        assert(Salesmen >= 1 && Salesmen <= Dimension);
        if (Salesmen == 1)
            ProblemType = TSP;
        Penalty = Penalty_CVRP;
    } else if (ProblemType == SOP || ProblemType == M1_PDTSP) {
        Constraint *Con;
        Node *Ni, *Nj;
        int n, k;
        OldDistance = Distance;
        Distance = Distance_SOP;
        if (ProblemType == M1_PDTSP) {
            for (i = 2; i < Dim; i++) {
                Ni = &NodeSet[i];
                for (k = n = 0; k < DemandDimension; k++) {
                    n = Ni->M_Demand[k];
                    if (n >= 0)
                        continue;
                    for (j = 2; j < Dim; j++) {
                        if (j == i)
                            continue;
                        Nj = &NodeSet[j];
                        if (Nj->M_Demand[k] == -n) {
                            Ni->C[j] = -1;
                            break;
                        }
                    }
                }
            }
        }
        for (j = 2; j < Dim; j++) {
            Nj = &NodeSet[j];
            for (i = 2; i < Dim; i++) {
                if (i != j && Nj->C[i] == -1) {
                    Ni = &NodeSet[i];
                    Con = (Constraint *) malloc(sizeof(Constraint));
                    Con->t1 = Ni;
                    Con->t2 = Nj;
                    Con->Suc = FirstConstraint;
                    FirstConstraint = Con;
                    Con->Next = Ni->FirstConstraint;
                    Ni->FirstConstraint = Con;
                }
            }
        }
        Salesmen = 1;
        Penalty = ProblemType == SOP ? Penalty_SOP : Penalty_M1_PDTSP;
    }
    if (ProblemType == TSPTW) {
        Salesmen = 1;
        Penalty = Penalty_TSPTW;
    } else
        TSPTW_Makespan = 0;
    if (Salesmen > 1) {
        if (Salesmen > Dim && MTSPMinSize > 0)
            eprintf("Too many salesmen/vehicles (>= DIMENSION)");
        MTSP2TSP();
    }
    if (ProblemType == STTSP)
        STTSP2TSP();
    if (ProblemType == ACVRP)
        Penalty = Penalty_ACVRP;
    else if (ProblemType == CCVRP)
        Penalty = Penalty_CCVRP;
    else if (ProblemType == CTSP)
        Penalty = Penalty_CTSP;
    else if (ProblemType == CVRPTW)
        Penalty = Penalty_CVRPTW;
    else if (ProblemType == MLP)
        Penalty = Penalty_MLP;
    else if (ProblemType == OVRP)
        Penalty = Penalty_OVRP;
    else if (ProblemType == PDTSP)
        Penalty = Penalty_PDTSP;
    else if (ProblemType == PDTSPF)
        Penalty = Penalty_PDTSPF;
    else if (ProblemType == PDTSPL)
        Penalty = Penalty_PDTSPL;
    else if (ProblemType == PDPTW)
        Penalty = Penalty_PDPTW;
    else if (ProblemType == ONE_PDTSP)
        Penalty = Penalty_1_PDTSP;
    else if (ProblemType == M_PDTSP)
        Penalty = Penalty_M_PDTSP;
    else if (ProblemType == M1_PDTSP)
        Penalty = Penalty_M1_PDTSP;
    else if (ProblemType == RCTVRP || ProblemType == RCTVRPTW)
        Penalty = Penalty_RCTVRP;
    else if (ProblemType == TRP)
        Penalty = Penalty_TRP;
    else if (ProblemType == TSPDL)
        Penalty = Penalty_TSPDL;
    else if (ProblemType == TSPPD)
        Penalty = Penalty_TSPPD;
    if (ProblemType == VRPB)
        Penalty = Penalty_VRPB;
    else if (ProblemType == VRPBTW)
        Penalty = Penalty_VRPBTW;
    else if (ProblemType == VRPPD)
        Penalty = Penalty_VRPPD;
    if (BWTSP_B > 0) {
        if (Penalty)
            eprintf("BWTSP not compatible with problem type %s\n", Type);
        ProblemType = BWTSP;
        free(Type);
        Type = Copy("BWTSP");
        Penalty = Penalty_BWTSP;
        if (BWTSP_L != INT_MAX)
            BWTSP_L *= Scale;
    }
    if (Penalty && (SubproblemSize > 0 || SubproblemTourFile))
        eprintf("Partitioning not implemented for constrained problems");
    Depot->DepotId = 1;
    for (i = Dim + 1; i <= DimensionSaved; i++)
        NodeSet[i].DepotId = i - Dim + 1;
    if (Dimension != DimensionSaved) {
        NodeSet[Depot->Id + DimensionSaved].DepotId = 1;
        for (i = Dim + 1; i <= DimensionSaved; i++)
            NodeSet[i + DimensionSaved].DepotId = i - Dim + 1;
    }
    if (Scale < 1)
        Scale = 1;
    else {
        Node *Ni = FirstNode;
        do {
            Ni->Earliest *= Scale;
            Ni->Latest *= Scale;
            Ni->ServiceTime *= Scale;
        } while ((Ni = Ni->Suc) != FirstNode);
        ServiceTime *= Scale;
        RiskThreshold *= Scale;
        if (DistanceLimit != DBL_MAX)
            DistanceLimit *= Scale;
    }
    if (ServiceTime != 0) {
        for (i = 1; i <= Dim; i++)
            NodeSet[i].ServiceTime = ServiceTime;
        Depot->ServiceTime = 0;
    }
    if (CostMatrix == 0 && Dimension <= MaxMatrixDimension &&
        Distance != 0 && Distance != Distance_1
        && Distance != Distance_EXPLICIT
        && Distance != Distance_LARGE && Distance != Distance_ATSP
        && Distance != Distance_MTSP && Distance != Distance_SPECIAL) {
        Node *Ni, *Nj;
        CostMatrix = (int *) calloc((size_t) Dim * (Dim - 1) / 2, sizeof(int));
        Ni = FirstNode->Suc;
        do {
            Ni->C =
                &CostMatrix[(size_t) (Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
            if (ProblemType != HPP || Ni->Id <= Dim)
                for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                    Ni->C[Nj->Id] = Fixed(Ni, Nj) ? 0 : Distance(Ni, Nj);
            else
                for (Nj = FirstNode; Nj != Ni; Nj = Nj->Suc)
                    Ni->C[Nj->Id] = 0;
        }
        while ((Ni = Ni->Suc) != FirstNode);
        c = 0;
        WeightType = EXPLICIT;
    }
    if (ProblemType == TSPTW ||
        ProblemType == CVRPTW || ProblemType == VRPBTW ||
        ProblemType == PDPTW || ProblemType == RCTVRPTW) {
        M = INT_MAX / 2 / Precision;
        for (i = 1; i <= Dim; i++) {
            Node *Ni = &NodeSet[i];
            for (j = 1; j <= Dim; j++) {
                Node *Nj = &NodeSet[j];
                if (Ni != Nj &&
                    Ni->Earliest + Ni->ServiceTime + Ni->C[j] > Nj->Latest)
                    Ni->C[j] = M;
            }
        }
    }
    C = WeightType == EXPLICIT ? C_EXPLICIT : C_FUNCTION;
    D = WeightType == EXPLICIT ? D_EXPLICIT : D_FUNCTION;
    if (ProblemType != CVRP && ProblemType != CVRPTW &&
        ProblemType != CTSP && ProblemType != STTSP &&
        ProblemType != TSP && ProblemType != ATSP) {
        M = INT_MAX / 2 / Precision;
        for (i = Dim + 1; i <= DimensionSaved; i++) {
            for (j = 1; j <= DimensionSaved; j++) {
                if (j == i)
                    continue;
                if (j == MTSPDepot || j > Dim)
                    NodeSet[i].C[j] = NodeSet[MTSPDepot].C[j] = M;
                NodeSet[i].C[j] = NodeSet[MTSPDepot].C[j];
                NodeSet[j].C[i] = NodeSet[j].C[MTSPDepot];
            }
        }
        if (ProblemType == CCVRP || ProblemType == OVRP)
            for (i = 1; i <= Dim; i++)
                NodeSet[i].C[MTSPDepot] = 0;
    }
    if (Precision > 1 && CostMatrix) {
        for (i = 2; i <= Dim; i++) {
            Node *N = &NodeSet[i];
            for (j = 1; j < i; j++)
                if (N->C[j] * Precision / Precision != N->C[j])
                    eprintf("PRECISION (= %d) is too large", Precision);
        }
    }
    if (SubsequentMoveType == 0) {
        SubsequentMoveType = MoveType;
        SubsequentMoveTypeSpecial = MoveTypeSpecial;
    }
    K = MoveType >= SubsequentMoveType || !SubsequentPatching ?
        MoveType : SubsequentMoveType;
    if (PatchingC > K)
        PatchingC = K;
    if (PatchingA > 1 && PatchingA >= PatchingC)
        PatchingA = PatchingC > 2 ? PatchingC - 1 : 1;
    if (NonsequentialMoveType == -1 ||
        NonsequentialMoveType > K + PatchingC + PatchingA - 1)
        NonsequentialMoveType = K + PatchingC + PatchingA - 1;
    if (PatchingC >= 1) {
        BestMove = BestSubsequentMove = BestKOptMove;
        if (!SubsequentPatching && SubsequentMoveType <= 5) {
            MoveFunction BestOptMove[] =
                { 0, 0, Best2OptMove, Best3OptMove,
                Best4OptMove, Best5OptMove
            };
            BestSubsequentMove = BestOptMove[SubsequentMoveType];
        }
    } else {
        MoveFunction BestOptMove[] = { 0, 0, Best2OptMove, Best3OptMove,
            Best4OptMove, Best5OptMove
        };
        BestMove = MoveType <= 5 ? BestOptMove[MoveType] : BestKOptMove;
        BestSubsequentMove = SubsequentMoveType <= 5 ?
            BestOptMove[SubsequentMoveType] : BestKOptMove;
    }
    if (MoveTypeSpecial)
        BestMove = BestSpecialOptMove;
    if (SubsequentMoveTypeSpecial)
        BestSubsequentMove = BestSpecialOptMove;
    if (ProblemType == HCP || ProblemType == HPP)
        MaxCandidates = 0;
    if (TraceLevel >= 1) {
        printff("done\n");
        PrintParameters();
    } else
        printff("PROBLEM_FILE = %s\n",
                ProblemFileName ? ProblemFileName : "");
    fclose(ProblemFile);
    if (InitialTourFileName)
        ReadTour(InitialTourFileName, &InitialTourFile);
    if (InputTourFileName)
        ReadTour(InputTourFileName, &InputTourFile);
    if (SubproblemTourFileName && SubproblemSize > 0)
        ReadTour(SubproblemTourFileName, &SubproblemTourFile);
    if (MergeTourFiles >= 1) {
        free(MergeTourFile);
        MergeTourFile = (FILE **) malloc(MergeTourFiles * sizeof(FILE *));
        for (i = 0; i < MergeTourFiles; i++)
            ReadTour(MergeTourFileName[i], &MergeTourFile[i]);
    }
    free(LastLine);
    LastLine = 0;
}

static int TwoDWeightType()
{
    if (Asymmetric)
        return 0;
    return WeightType == EUC_2D || WeightType == MAX_2D ||
        WeightType == MAN_2D || WeightType == CEIL_2D ||
        WeightType == FLOOR_2D ||
        WeightType == GEO || WeightType == GEOM ||
        WeightType == GEO_MEEUS || WeightType == GEOM_MEEUS ||
        WeightType == ATT || WeightType == TOR_2D ||
        (WeightType == SPECIAL && CoordType == TWOD_COORDS);
}

static int ThreeDWeightType()
{
    if (Asymmetric)
        return 0;
    return WeightType == EUC_3D || WeightType == MAX_3D ||
        WeightType == MAN_3D || WeightType == CEIL_3D ||
        WeightType == FLOOR_3D || WeightType == TOR_3D ||
        WeightType == XRAY1 || WeightType == XRAY2 ||
        (WeightType == SPECIAL && CoordType == THREED_COORDS);
}

static void CheckSpecificationPart()
{
    if (ProblemType == -1)
        eprintf("TYPE is missing");
    if (Dimension < 3)
        eprintf("DIMENSION < 3 or not specified");
    if (WeightType == -1 && !Asymmetric && ProblemType != HCP &&
        ProblemType != HPP && !EdgeWeightType && ProblemType != STTSP)
        eprintf("EDGE_WEIGHT_TYPE is missing");
    if (WeightType == EXPLICIT && WeightFormat == -1 && !EdgeWeightFormat)
        eprintf("EDGE_WEIGHT_FORMAT is missing");
    if (WeightType == EXPLICIT && WeightFormat == FUNCTION)
        eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
    if (WeightType != EXPLICIT &&
        (WeightType != SPECIAL || CoordType != NO_COORDS) &&
        WeightType != -1 && WeightFormat != -1 && WeightFormat != FUNCTION)
        eprintf("Conflicting EDGE_WEIGHT_TYPE and EDGE_WEIGHT_FORMAT");
    if ((ProblemType == ATSP || ProblemType == SOP) &&
        WeightType != EXPLICIT && WeightType != -1)
        eprintf("Conflicting TYPE and EDGE_WEIGHT_TYPE");
    if (CandidateSetType == DELAUNAY && !TwoDWeightType() &&
        MaxCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = DELAUNAY");
    if (CandidateSetType == QUADRANT && !TwoDWeightType() &&
        !ThreeDWeightType() && MaxCandidates + ExtraCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for CANDIDATE_SET_TYPE = QUADRANT");
    if (ExtraCandidateSetType == QUADRANT && !TwoDWeightType() &&
        !ThreeDWeightType() && ExtraCandidates > 0)
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for EXTRA_CANDIDATE_SET_TYPE = "
             "QUADRANT");
    if (InitialTourAlgorithm == QUICK_BORUVKA && !TwoDWeightType() &&
        !ThreeDWeightType())
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
             "QUICK-BORUVKA");
    if (InitialTourAlgorithm == SIERPINSKI && !TwoDWeightType())
        eprintf
            ("Illegal EDGE_WEIGHT_TYPE for INITIAL_TOUR_ALGORITHM = "
             "SIERPINSKI");
    if (DelaunayPartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for DELAUNAY specification");
    if (KarpPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for KARP specification");
    if (KCenterPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for K-CENTER specification");
    if (KMeansPartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for K-MEANS specification");
    if (MoorePartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for MOORE specification");
    if (RohePartitioning && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for ROHE specification");
    if (SierpinskiPartitioning && !TwoDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for SIERPINSKI specification");
    if (SubproblemBorders && !TwoDWeightType() && !ThreeDWeightType())
        eprintf("Illegal EDGE_WEIGHT_TYPE for BORDERS specification");
    if (InitialTourAlgorithm == MTSP_ALG && Asymmetric)
        eprintf("INTIAL_TOUR_ALGORITHM = MTSP is not applicable for "
                "asymetric problems");
}

static char *Copy(char *S)
{
    char *Buffer;

    if (!S || strlen(S) == 0)
        return 0;
    Buffer = (char *) malloc(strlen(S) + 1);
    strcpy(Buffer, S);
    return Buffer;
}

static void CreateNodes()
{
    Node *Prev = 0, *N = 0;
    int i;

    if (Dimension <= 0)
        eprintf("DIMENSION is not positive (or not specified)");
    if (Asymmetric) {
        Dim = DimensionSaved;
        DimensionSaved = Dimension + Salesmen - 1;
        Dimension = 2 * DimensionSaved;
    } else if (ProblemType == HPP) {
        Dimension++;
        if (Dimension > MaxMatrixDimension)
            eprintf("DIMENSION too large in HPP problem");
    }
    NodeSet = (Node *) calloc(Dimension + 1, sizeof(Node));
    for (i = 1; i <= Dimension; i++, Prev = N) {
        N = &NodeSet[i];
        if (i == 1)
            FirstNode = N;
        else
            Link(Prev, N);
        N->Id = N->OriginalId = i;
        if (MergeTourFiles >= 1)
            N->MergeSuc = (Node **) calloc(MergeTourFiles, sizeof(Node *));
        N->Earliest = 0;
        N->Latest = INT_MAX;
    }
    Link(N, FirstNode);
}


static int FixEdge(Node * Na, Node * Nb)
{
    if (!Na->FixedTo1 || Na->FixedTo1 == Nb)
        Na->FixedTo1 = Nb;
    else if (!Na->FixedTo2 || Na->FixedTo2 == Nb)
        Na->FixedTo2 = Nb;
    else
        return 0;
    if (!Nb->FixedTo1 || Nb->FixedTo1 == Na)
        Nb->FixedTo1 = Na;
    else if (!Nb->FixedTo2 || Nb->FixedTo1 == Na)
        Nb->FixedTo2 = Na;
    else
        return 0;
    return 1;
}

static void Read_NAME()
{
    if (!(Name = Copy(strtok(0, Delimiters))))
        eprintf("NAME: string expected");
}

static void Read_BACKHAUL_SECTION()
{
    int Id;

    while (fscanint(ProblemFile, &Id) && Id != -1) {
        if (Id <= 0 || Id > Dim)
            eprintf("BACKHAUL_SECTION: Node number out of range: %d", Id);
        NodeSet[Id].Backhaul = 1;
        NodeSet[Id + DimensionSaved].Backhaul = 1;
    }
}

static void Read_CAPACITY()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &Capacity))
        eprintf("CAPACITY: Integer expected");
}

static void Read_CTSP_SET_SECTION()
{   
    Node *N;
    int Id, n, *ColorUsed;
    
    N = FirstNode;
    do {
        N->Color = 0;
    } while ((N = N->Suc) != FirstNode);
    ColorUsed = (int *) calloc(Salesmen + 1, sizeof(int));
    while (fscanf(ProblemFile, "%d", &Id) > 0) {
        if (Id < 1 || Id > Salesmen)
            eprintf("CTSP_SET_SECTION: Color number %d outside range", Id);
        if (ColorUsed[Id])
            eprintf("CTSP_SET_SECTION: Color number %d used twice", Id);
        ColorUsed[Id] = 1;
        for (;;) {
            if (fscanf(ProblemFile, "%d", &n) != 1)
                eprintf("CTSP_SET_SECTION: Missing -1");
            if (n == -1)
                break; 
             if (n < 1 || n > DimensionSaved)
                 eprintf("CTSP_SET_SECTION: Node %d outside range", n);
             N = &NodeSet[n];
             if (N->Color != 0 && N->Color != Id) 
                 eprintf("CTSP_SET_SECTION: Node %d in two sets", n);
             if (N == Depot)
                 eprintf("CTSP_SET_SECTION: Depot %d occurs in set %d", n, Id);
             N->Color = Id;
        }
    }
    free(ColorUsed);
}

static void Read_DEMAND_DIMENSION()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &DemandDimension))
        eprintf("DIMENSION_DIMENSION: Integer expected");
    if (DemandDimension < 0)
        eprintf("DIMENSION_DIMENSION: < 0");
}

static void Read_DEMAND_SECTION()
{
    int Id, Demand, i, k;
    Node *N;

    for (i = 1; i <= Dim; i++) {
        fscanint(ProblemFile, &Id);
        if (Id <= 0 || Id > Dim)
            eprintf("DEMAND_SECTION: Node number out of range: %d", Id);
        N = &NodeSet[Id];
        if (DemandDimension > 1) {
            N->M_Demand = (int *) malloc(DemandDimension * sizeof(int));
            for (k = 0; k < DemandDimension; k++) {
                if (!fscanint(ProblemFile, &Demand))
                    eprintf("DEMAND_SECTION: Missing demand for node %d",
                            Id);
                N->M_Demand[k] = Demand;
            }
        } else if (!fscanint(ProblemFile, &N->Demand))
            eprintf("DEMAND_SECTION: Missing demand for node %d", Id);
    }
}

static void Read_DEPOT_SECTION()
{
    int i;
    if (!fscanint(ProblemFile, &MTSPDepot))
        eprintf("DEPOT_SECTION: Integer expected");
    else if (MTSPDepot <= 0)
        eprintf("DEPOT_SECTION: Positive value expected");
    if (fscanint(ProblemFile, &i) && i != -1)
        eprintf("DEPOT_SECTION: Only one depot allowed");
}

static void Read_DIMENSION()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &Dimension))
        eprintf("DIMENSION: Integer expected");
    if (Dimension < 0)
        eprintf("DIMENSION: < 0");
    DimensionSaved = Dim = Dimension;
}

static void Read_DISPLAY_DATA_SECTION()
{
    Node *N;
    int Id, i;

    CheckSpecificationPart();
    if (ProblemType == HPP)
        Dimension--;
    if (!DisplayDataType || strcmp(DisplayDataType, "TWOD_DISPLAY"))
        eprintf
            ("DISPLAY_DATA_SECTION conflicts with DISPLAY_DATA_TYPE: %s",
             DisplayDataType);
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    for (i = 1; i <= Dim; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("DIPLAY_DATA_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dimension)
            eprintf("DIPLAY_DATA_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("DIPLAY_DATA_SECTION: Node number occurs twice: %d",
                    N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%lf", &N->X))
            eprintf("DIPLAY_DATA_SECTION: Missing X-coordinate");
        if (!fscanf(ProblemFile, "%lf", &N->Y))
            eprintf("DIPLAY_DATA_SECTION: Missing Y-coordinate");
        if (CoordType == THREED_COORDS
            && !fscanf(ProblemFile, "%lf", &N->Z))
            eprintf("DIPLAY_DATA_SECTION: Missing Z-coordinate");
    }
    N = FirstNode;
    do
        if (!N->V && N->Id <= Dim)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("DIPLAY_DATA_SECTION: No coordinates given for node %d",
                N->Id);
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_DISPLAY_DATA_TYPE()
{
    unsigned int i;

    if (!(DisplayDataType = Copy(strtok(0, Delimiters))))
        eprintf("DISPLAY_DATA_TYPE: string expected");
    for (i = 0; i < strlen(DisplayDataType); i++)
        DisplayDataType[i] = (char) toupper(DisplayDataType[i]);
    if (strcmp(DisplayDataType, "COORD_DISPLAY") &&
        strcmp(DisplayDataType, "TWOD_DISPLAY") &&
        strcmp(DisplayDataType, "NO_DISPLAY"))
        eprintf("Unknown DISPLAY_DATA_TYPE: %s", DisplayDataType);
}

static void Read_DISTANCE()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%lf", &DistanceLimit))
        eprintf("DISTANCE: real expected");
}

static void Read_DRAFT_LIMIT_SECTION()
{
    int Id, i;
    Node *N;

    for (i = 1; i <= Dim; i++) {
        fscanint(ProblemFile, &Id);
        if (Id <= 0 || Id > Dim)
            eprintf("DRAFT_LIMIT_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (!fscanint(ProblemFile, &N->DraftLimit))
            eprintf("DRAFT_LIMIT_SECTION: Missing draft limit for node %d",
                    Id);
    }
}

static void Read_EDGE_DATA_FORMAT()
{
    unsigned int i;

    if (!(EdgeDataFormat = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_DATA_FORMAT: string expected");
    for (i = 0; i < strlen(EdgeDataFormat); i++)
        EdgeDataFormat[i] = (char) toupper(EdgeDataFormat[i]);
    if (strcmp(EdgeDataFormat, "EDGE_LIST") &&
        strcmp(EdgeDataFormat, "ADJ_LIST"))
        eprintf("Unknown EDGE_DATA_FORMAT: %s", EdgeDataFormat);
    if (SubproblemTourFileName)
        eprintf("EDGE_DATA_FORMAT "
                "cannot be used together with SUBPROBLEM_TOUR_FILE");
}

static void Read_EDGE_DATA_SECTION()
{
    Node *Ni, *Nj;
    int i, j, W = 0, WithWeights = 0, FirstLine = 1;
    double w = 0;
    char *Line;

    CheckSpecificationPart();
    if (!EdgeDataFormat)
        eprintf("Missing EDGE_DATA_FORMAT specification");
    if (!FirstNode)
        CreateNodes();
    if (ProblemType == HPP)
        Dimension--;
    if (Scale < 0)
        Scale = 1;
    if (!strcmp(EdgeDataFormat, "EDGE_LIST")) {
        Line = ReadLine(ProblemFile);
        if (sscanf(Line, "%d %d %lf\n", &i, &j, &w) == 3)
            WithWeights = 1;
        W = round(Scale * w);
        while (i != -1) {
            if (i <= 0 ||
                i > (!Asymmetric ? Dimension : Dimension / 2))
                eprintf("EDGE_DATA_SECTION: Node number out of range: %d", i);
            if (!FirstLine)
                fscanint(ProblemFile, &j);
            if (j <= 0
                || j > (!Asymmetric ? Dimension : Dimension / 2))
                eprintf("EDGE_DATA_SECTION: Node number out of range: %d",
                        j);
            if (i == j)
                eprintf("EDGE_DATA_SECTION: Illegal edge: %d to %d",
                        i, j);
            if (Asymmetric)
                j += Dimension / 2;
            Ni = &NodeSet[i];
            Nj = &NodeSet[j];
            if (WithWeights) {
                if (!FirstLine) {
                    fscanf(ProblemFile, "%lf", &w);
                    W = round(Scale * w);
                }
                W *= Precision;
            }
            AddCandidate(Ni, Nj, W, 1);
            AddCandidate(Nj, Ni, W, 1);
            FirstLine = 0;
            if (!fscanint(ProblemFile, &i))
                i = -1;
        }
    } else if (!strcmp(EdgeDataFormat, "ADJ_LIST")) {
        if (!fscanint(ProblemFile, &i))
            i = -1;
        while (i != -1) {
            if (i <= 0 ||
                (!Asymmetric ? Dimension : Dimension / 2))
                eprintf
                ("EDGE_DATA_SECTION: Node number out of range: %d",
                 i);
            Ni = &NodeSet[i];
            fscanint(ProblemFile, &j);
            while (j != -1) {
                if (j <= 0 ||
                    (ProblemType != ATSP ? Dimension : Dimension / 2))
                    eprintf
                    ("EDGE_DATA_SECTION: Node number out of range: %d",
                     j);
                if (i == j)
                    eprintf("EDGE_DATA_SECTION: Illgal edge: %d to %d",
                            i, j);
                if (Asymmetric)
                    j += Dimension / 2;
                Nj = &NodeSet[j];
                AddCandidate(Ni, Nj, 0, 1);
                AddCandidate(Nj, Ni, 0, 1);
                fscanint(ProblemFile, &j);
            }
            fscanint(ProblemFile, &i);
        }
    } else
        eprintf("EDGE_DATA_SECTION: No EDGE_DATA_FORMAT specified");
    if (ProblemType == HPP)
        Dimension++;
    if (Asymmetric) {
        for (i = 1; i <= DimensionSaved; i++)
            FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
    }
    WeightType = 1;
    if (ProblemType != STTSP)
        MaxCandidates = ExtraCandidates = 0;
    Distance = WithWeights ? Distance_LARGE : Distance_1;
}

static void Read_EDGE_WEIGHT_FORMAT()
{
    unsigned int i;

    if (!(EdgeWeightFormat = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_WEIGHT_FORMAT: string expected");
    for (i = 0; i < strlen(EdgeWeightFormat); i++)
        EdgeWeightFormat[i] = (char) toupper(EdgeWeightFormat[i]);
    if (!strcmp(EdgeWeightFormat, "FUNCTION"))
        WeightFormat = FUNCTION;
    else if (!strcmp(EdgeWeightFormat, "FULL_MATRIX"))
        WeightFormat = FULL_MATRIX;
    else if (!strcmp(EdgeWeightFormat, "UPPER_ROW"))
        WeightFormat = UPPER_ROW;
    else if (!strcmp(EdgeWeightFormat, "LOWER_ROW"))
        WeightFormat = LOWER_ROW;
    else if (!strcmp(EdgeWeightFormat, "UPPER_DIAG_ROW"))
        WeightFormat = UPPER_DIAG_ROW;
    else if (!strcmp(EdgeWeightFormat, "LOWER_DIAG_ROW"))
        WeightFormat = LOWER_DIAG_ROW;
    else if (!strcmp(EdgeWeightFormat, "UPPER_COL"))
        WeightFormat = UPPER_COL;
    else if (!strcmp(EdgeWeightFormat, "LOWER_COL"))
        WeightFormat = LOWER_COL;
    else if (!strcmp(EdgeWeightFormat, "UPPER_DIAG_COL"))
        WeightFormat = UPPER_DIAG_COL;
    else if (!strcmp(EdgeWeightFormat, "LOWER_DIAG_COL"))
        WeightFormat = LOWER_DIAG_COL;
    else
        eprintf("Unknown EDGE_WEIGHT_FORMAT: %s", EdgeWeightFormat);
}

static void Read_EDGE_WEIGHT_SECTION()
{
    Node *Ni;
    int i, j, n, W;
    double w;

    if (ProblemType == SOP && ProblemType != M1_PDTSP) {
        fscanint(ProblemFile, &n);
        if (n != Dimension)
            eprintf("SOP: DIMENSION != n (%d != %d)", Dimension, n);
    } else
        n = Dimension;
    CheckSpecificationPart();
    if (!FirstNode)
        CreateNodes();
    if (!Asymmetric) {
        CostMatrix = (int *) calloc((size_t) Dimension * (Dimension - 1) / 2,
                                    sizeof(int));
        Ni = FirstNode->Suc;
        do {
            Ni->C =
                &CostMatrix[(size_t) (Ni->Id - 1) * (Ni->Id - 2) / 2] - 1;
        }
        while ((Ni = Ni->Suc) != FirstNode);
    } else {
        n = Dimension / 2;
        CostMatrix = (int *) calloc((size_t) n * n, sizeof(int));
        for (Ni = FirstNode; Ni->Id <= n; Ni = Ni->Suc)
            Ni->C = &CostMatrix[(size_t) (Ni->Id - 1) * n] - 1;
    }
    if (ProblemType == HPP)
        Dimension--;
    switch (WeightFormat) {
    case FULL_MATRIX:
        for (i = 1; i <= Dim; i++) {
            Ni = &NodeSet[i];
            for (j = 1; j <= Dim; j++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                W = round(Scale * w);
                if (j != i && W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                if (Asymmetric) {
                    Ni->C[j] = W;
                    if (j != i && W > M)
                        M = W;
                } else if (j < i)
                    Ni->C[j] = W;
            }
        }
        break;
    case UPPER_ROW:
        for (i = 1; i < Dim; i++) {
            for (j = i + 1; j <= Dim; j++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[j].C[i] = W;
                if (Asymmetric) {
                    NodeSet[i].C[j] = W;
                    if (j != i && W > M)
                        M = W;
                }
            }
        }
        break;
    case LOWER_ROW:
        for (i = 2; i <= Dim; i++) {
            for (j = 1; j < i; j++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                W = round(Scale * w);
                NodeSet[i].C[j] = W;
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                if (Asymmetric) {
                    NodeSet[j].C[i] = W;
                    if (j != i && W > M)
                        M = W;
                }
            }
        }
        break;
    case UPPER_DIAG_ROW:
        for (i = 1; i <= Dim; i++) {
            for (j = i; j <= Dim; j++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[j].C[i] = W;
                if (Asymmetric) {
                    NodeSet[j].C[i] = W;
                    if (W > M)
                        M = W;
                }
            }
        }
        break;
    case LOWER_DIAG_ROW:
        for (i = 1; i <= Dim; i++) {
            for (j = 1; j <= i; j++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                if (j != i)
                    NodeSet[i].C[j] = W;
                if (Asymmetric) {
                    NodeSet[j].C[i] = W;
                    if (W > M)
                        M = W;
                }
            }
        }
        break;
    case UPPER_COL:
        for (j = 2; j <= Dim; j++) {
            for (i = 1; i < j; i++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[j].C[i] = W;
                if (Asymmetric) {
                    NodeSet[i].C[j] = W;
                    if (j != i && W > M)
                        M = W;
                }
            }
        }
        break;
    case LOWER_COL:
        for (j = 1; j < Dim; j++) {
            for (i = j + 1; i <= Dim; i++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[i].C[j] = W;
                if (Asymmetric) {
                    NodeSet[j].C[i] = W;
                    if (j != i && W > M)
                        M = W;
                }
            }
        }
        break;
    case UPPER_DIAG_COL:
        for (j = 1; j <= Dim; j++) {
            for (i = 1; i <= j; i++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[j].C[i] = W;
                if (Asymmetric) {
                    NodeSet[i].C[j] = W;
                    if (W > M)
                        M = W;
                }
            }
        }
        break;
    case LOWER_DIAG_COL:
        for (j = 1; j <= Dim; j++) {
            for (i = j; i <= Dim; i++) {
                if (!fscanf(ProblemFile, "%lf", &w))
                    eprintf("EDGE_WEIGHT_SECTION: Missing weight");
                if (j == i)
                    continue;
                W = round(Scale * w);
                if (W > INT_MAX / 2 / Precision)
                    eprintf("EDGE_WEIGHT_SECTION: "
                            "Weight %d > INT_MAX / 2 / PRECISION", W);
                NodeSet[i].C[j] = W;
                if (Asymmetric) {
                    NodeSet[j].C[i] = W;
                    if (W > M)
                        M = W;
                }
            }
        }
        break;
    }
    if (ProblemType == HPP)
        Dimension++;
    if (Asymmetric) {
        for (i = 1; i <= DimensionSaved; i++)
            FixEdge(&NodeSet[i], &NodeSet[i + DimensionSaved]);
        if (ProblemType == SOP || ProblemType == M1_PDTSP)
            NodeSet[n].C[1] = 0;
        Distance = Distance_ATSP;
        WeightType = -1;
    }
}

static void Read_EDGE_WEIGHT_TYPE()
{
    unsigned int i;

    if (!(EdgeWeightType = Copy(strtok(0, Delimiters))))
        eprintf("EDGE_WEIGHT_TYPE: string expected");
    for (i = 0; i < strlen(EdgeWeightType); i++)
        EdgeWeightType[i] = (char) toupper(EdgeWeightType[i]);
    if (!strcmp(EdgeWeightType, "ATT")) {
        WeightType = ATT;
        Distance = Distance_ATT;
        c = c_ATT;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "CEIL_2D")) {
        WeightType = CEIL_2D;
        Distance = Distance_CEIL_2D;
        c = c_CEIL_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "CEIL_3D")) {
        WeightType = CEIL_3D;
        Distance = Distance_CEIL_3D;
        c = c_CEIL_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "EUC_2D") ||
               !strcmp(EdgeWeightType, "EXACT_2D")) {
        WeightType = EUC_2D;
        Distance = Distance_EUC_2D;
        c = c_EUC_2D;
        CoordType = TWOD_COORDS;
        if (Scale == -1 && !strcmp(EdgeWeightType, "EXACT_2D"))
            Scale = 1000;
    } else if (!strcmp(EdgeWeightType, "EUC_3D") ||
               !strcmp(EdgeWeightType, "EXACT_3D")) {
        WeightType = EUC_3D;
        Distance = Distance_EUC_3D;
        c = c_EUC_3D;
        CoordType = THREED_COORDS;
        if (Scale == -1 && !strcmp(EdgeWeightType, "EXACT_3D"))
            Scale = 1000;
    } else if (!strcmp(EdgeWeightType, "EXPLICIT")) {
        WeightType = EXPLICIT;
        Distance = Distance_EXPLICIT;
        if (Scale < 1)
            Scale = 1;
    } else if (!strcmp(EdgeWeightType, "FLOOR_2D")) {
        WeightType = FLOOR_2D;
        Distance = Distance_FLOOR_2D;
        c = c_FLOOR_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "FLOOR_3D")) {
        WeightType = FLOOR_3D;
        Distance = Distance_FLOOR_3D;
        c = c_FLOOR_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAN_2D")) {
        WeightType = MAN_2D;
        Distance = Distance_MAN_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAN_3D")) {
        WeightType = MAN_3D;
        Distance = Distance_MAN_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAX_2D")) {
        WeightType = MAX_2D;
        Distance = Distance_MAX_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "MAX_3D")) {
        WeightType = MAX_3D;
        Distance = Distance_MAX_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEO")) {
        WeightType = GEO;
        Distance = Distance_GEO;
        c = c_GEO;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEOM")) {
        WeightType = GEOM;
        Distance = Distance_GEOM;
        c = c_GEOM;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEO_MEEUS")) {
        WeightType = GEO_MEEUS;
        Distance = Distance_GEO_MEEUS;
        c = c_GEO_MEEUS;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "GEOM_MEEUS")) {
        WeightType = GEOM_MEEUS;
        Distance = Distance_GEOM_MEEUS;
        c = c_GEOM_MEEUS;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "TOR_2D")) {
        WeightType = TOR_2D;
        Distance = Distance_TOR_2D;
        CoordType = TWOD_COORDS;
    } else if (!strcmp(EdgeWeightType, "TOR_3D")) {
        WeightType = TOR_3D;
        Distance = Distance_TOR_3D;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "XRAY1")) {
        WeightType = XRAY1;
        Distance = Distance_XRAY1;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "XRAY2")) {
        WeightType = XRAY2;
        Distance = Distance_XRAY2;
        CoordType = THREED_COORDS;
    } else if (!strcmp(EdgeWeightType, "SPECIAL")) {
        WeightType = SPECIAL;
        Distance = Distance_SPECIAL;
    } else
        eprintf("Unknown EDGE_WEIGHT_TYPE: %s", EdgeWeightType);
}

static void Read_FIXED_EDGES_SECTION()
{
    Node *Ni, *Nj, *N, *NPrev = 0, *NNext;
    int i, j, Count = 0;

    CheckSpecificationPart();
    if (!FirstNode)
        CreateNodes();
    if (ProblemType == HPP)
        Dimension--;
    if (!fscanint(ProblemFile, &i))
        i = -1;
    while (i != -1) {
        if (i <= 0 || i > (Asymmetric ? Dimension / 2 : Dimension))
            eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    i);
        fscanint(ProblemFile, &j);
        if (j <= 0 || j > (Asymmetric ? Dimension / 2 : Dimension))
            eprintf("FIXED_EDGES_SECTION: Node number out of range: %d",
                    j);
        if (i == j)
            eprintf("FIXED_EDGES_SECTION: Illegal edge: %d to %d", i, j);
        Ni = &NodeSet[i];
        Nj = &NodeSet[Asymmetric ? j + Dimension / 2 : j];
        if (!FixEdge(Ni, Nj))
            eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
        /* Cycle check */
        N = Ni;
        Count = 0;
        do {
            NNext = N->FixedTo1 != NPrev ? N->FixedTo1 : N->FixedTo2;
            NPrev = N;
            Count++;
        } while ((N = NNext) && N != Ni);
        if (N == Ni && Count != Dimension)
            eprintf("FIXED_EDGES_SECTION: Illegal fix: %d to %d", i, j);
        if (!fscanint(ProblemFile, &i))
            i = -1;
    }
    if (ProblemType == HPP)
        Dimension++;
}

static void Read_GRID_SIZE()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%lf", &GridSize))
        eprintf("GRID_SIZE: real expected");
    if (GridSize < 0)
        eprintf("GRID_SIZE: non-negative real expected");
}

static void Read_NODE_COORD_SECTION()
{
    Node *N;
    int Id, i;

    CheckSpecificationPart();
    if (CoordType != TWOD_COORDS && CoordType != THREED_COORDS)
        eprintf("NODE_COORD_SECTION conflicts with NODE_COORD_TYPE: %s",
                NodeCoordType);
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    if (ProblemType == HPP)
        Dimension--;
    for (i = 1; i <= Dim; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("NODE_COORD_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dimension)
            eprintf("NODE_COORD_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("NODE_COORD_SECTION: Node number occurs twice: %d",
                    N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%lf", &N->X))
            eprintf("NODE_COORD_SECTION: Missing X-coordinate");
        if (!fscanf(ProblemFile, "%lf", &N->Y))
            eprintf("NODE_COORD_SECTION: Missing Y-coordinate");
        if (CoordType == THREED_COORDS
            && !fscanf(ProblemFile, "%lf", &N->Z))
            eprintf("NODE_COORD_SECTION: Missing Z-coordinate");
        if (Name && !strcmp(Name, "d657")) {
            N->X = (float) N->X;
            N->Y = (float) N->Y;
        }
    }
    N = FirstNode;
    do
        if (!N->V && N->Id <= Dim)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("NODE_COORD_SECTION: No coordinates given for node %d",
                N->Id);
    if (ProblemType == HPP)
        Dimension++;
    if (Asymmetric)
        Convert2FullMatrix();
}

static void Read_NODE_COORD_TYPE()
{
    unsigned int i;

    if (!(NodeCoordType = Copy(strtok(0, Delimiters))))
        eprintf("NODE_COORD_TYPE: string expected");
    for (i = 0; i < strlen(NodeCoordType); i++)
        NodeCoordType[i] = (char) toupper(NodeCoordType[i]);
    if (!strcmp(NodeCoordType, "TWOD_COORDS"))
        CoordType = TWOD_COORDS;
    else if (!strcmp(NodeCoordType, "THREED_COORDS"))
        CoordType = THREED_COORDS;
    else if (!strcmp(NodeCoordType, "NO_COORDS"))
        CoordType = NO_COORDS;
    else
        eprintf("Unknown NODE_COORD_TYPE: %s", NodeCoordType);
}

static void Read_PICKUP_AND_DELIVERY_SECTION()
{
    int Id, i;
    Node *N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    for (i = 1; i <= Dim; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("PICKUP_AND_DELIVERY_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dim)
            eprintf
                ("PICKUP_AND_DELIVERY_SECTION: Node number out of range: %d",
                 Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Node number occurs twice: %d", N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%d %lf %lf %lf %d %d",
                    &N->Demand, &N->Earliest, &N->Latest, &N->ServiceTime,
                    &N->Pickup, &N->Delivery))
            eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Missing data for node %d", N->Id);
        if (N->ServiceTime < 0)
            eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Negative Service Time for node %d", N->Id);
        if (N->Earliest > N->Latest)
            eprintf("PICKUP_AND_DELIVERY_SECTION: "
                    "Earliest > Latest for node %d", N->Id);
    }
    N = FirstNode;
    do
        if (!N->V && N->Id <= Dim)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("PICKUP_AND_DELIVERY_SECTION: No data given for node %d",
                N->Id);
    if (ProblemType != VRPPD) {
        do {
            if (N->Delivery) {
                if (NodeSet[N->Delivery].Pickup != N->Id ||
                    N->Delivery == N->Id)
                    eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Illegal pairing for node %d", N->Id);
                if (N->Demand < 0)
                    eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Negative demand for delivery node %d", N->Id);
            } else if (N->Pickup) {
                if (NodeSet[N->Pickup].Delivery != N->Id
                    || N->Pickup == N->Id)
                    eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Illegal pairing for node %d", N->Id);
                if (N->Demand > 0)
                    eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Positive demand for pickup node %d", N->Id);
                if (N->Demand + NodeSet[N->Pickup].Demand)
                    eprintf("PICKUP_AND_DELIVERY_SECTION: "
                            "Demand for pickup node %d and demand for delivery "
                            "node %d does not sum to zero", N->Id,
                            N->Pickup);
            }
        } while ((N = N->Suc) != FirstNode);
    }
}

static void Read_REQUIRED_NODES_SECTION(void)
{
    int i;

    CheckSpecificationPart();
    if (!FirstNode)
        CreateNodes();
    if (!fscanint(ProblemFile, &i))
        i = -1;
    while (i != -1) {
        if (i <= 0 || i > Dimension)
            eprintf("REQUIRED_NODES__SECTION: Node number out of range: %d",
                    i);
        NodeSet[i].Required = 1;
        if (!fscanint(ProblemFile, &i))
            i = -1;
    }
}

static void Read_TIME_WINDOW_SECTION()
{
    int Id, i;
    Node *N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    for (i = 1; i <= Dim; i++) {
        if (!fscanint(ProblemFile, &Id))
            eprintf("TIME_WINDOW_SECTION: Missing nodes");
        if (Id <= 0 || Id > Dim)
            eprintf("TIME_WINDOW_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (N->V == 1)
            eprintf("TIME_WINDOW_SECTION: Node number occurs twice: %d",
                    N->Id);
        N->V = 1;
        if (!fscanf(ProblemFile, "%lf", &N->Earliest))
            eprintf("TIME_WINDOW_SECTION: Missing earliest time");
        if (!fscanf(ProblemFile, "%lf", &N->Latest))
            eprintf("TIME_WINDOW_SECTION: Missing latest time");
        if (N->Earliest > N->Latest)
            printff("%d: %f %f\n", N->Id, N->Earliest, N->Latest);
        if (N->Earliest > N->Latest)
            eprintf("TIME_WINDOW_SECTION: Earliest > Latest for node %d",
                    N->Id);
    }
    N = FirstNode;
    do
        if (!N->V && N->Id <= Dim)
            break;
    while ((N = N->Suc) != FirstNode);
    if (!N->V)
        eprintf("TIME_WINDOW_SECTION: No time window given for node %d",
                N->Id);
}

static void Read_TOUR_SECTION(FILE ** File)
{
    Node *First = 0, *Last = 0, *N, *Na;
    int i, k;

    if (TraceLevel >= 1) {
        printff("Reading ");
        if (File == &InitialTourFile)
            printff("INITIAL_TOUR_FILE: \"%s\" ... ", InitialTourFileName);
        else if (File == &InputTourFile)
            printff("INPUT_TOUR_FILE: \"%s\" ... ", InputTourFileName);
        else if (File == &SubproblemTourFile)
            printff("SUBPROBLEM_TOUR_FILE: \"%s\" ... ",
                    SubproblemTourFileName);
        else
            for (i = 0; i < MergeTourFiles; i++)
                if (File == &MergeTourFile[i])
                    printff("MERGE_TOUR_FILE: \"%s\" ... ",
                            MergeTourFileName[i]);
    }
    if (!FirstNode)
        CreateNodes();
    N = FirstNode;
    do
        N->V = 0;
    while ((N = N->Suc) != FirstNode);
    if (ProblemType == HPP)
        Dimension--;
    if (Asymmetric)
        Dimension = DimensionSaved;
    int b = 0;
    if (!fscanint(*File, &i))
        i = -1;
    else if (i == 0) {
        b = 1;
        i++;
    }
    for (k = 0; k <= Dimension && i != -1; k++) {
        if (i <= 0 || i > Dimension)
            eprintf("TOUR_SECTION: Node number out of range: %d", i);
        N = &NodeSet[i];
        if (N->V == 1 && k != Dimension)
            eprintf("TOUR_SECTION: Node number occurs twice: %d", N->Id);
        N->V = 1;
        if (k == 0)
            First = Last = N;
        else {
            if (Asymmetric) {
                Na = N + Dimension;
                Na->V = 1;
            } else
                Na = 0;
            if (File == &InitialTourFile) {
                if (!Na)
                    Last->InitialSuc = N;
                else {
                    Last->InitialSuc = Na;
                    Na->InitialSuc = N;
                }
            } else if (File == &InputTourFile) {
                if (!Na)
                    Last->InputSuc = N;
                else {
                    Last->InputSuc = Na;
                    Na->InputSuc = N;
                }
            } else if (File == &SubproblemTourFile) {
                if (!Na)
                    (Last->SubproblemSuc = N)->SubproblemPred = Last;
                else {
                    (Last->SubproblemSuc = Na)->SubproblemPred = Last;
                    (Na->SubproblemSuc = N)->SubproblemPred = Na;
                }
            } else {
                for (i = 0; i < MergeTourFiles; i++) {
                    if (File == &MergeTourFile[i]) {
                        if (!Na) {
                            Last->MergeSuc[i] = N;
                            if (i == 0)
                                N->MergePred = Last;
                        } else {
                            Last->MergeSuc[i] = Na;
                            Na->MergeSuc[i] = N;
                            if (i == 0) {
                                Na->MergePred = Last;
                                N->MergePred = Na;
                            }
                        }
                    }
                }
            }
            Last = N;
        }
        if (k < Dimension) {
            fscanint(*File, &i);
            if (b)
                if (i >= 0)
                    i++;
        }
        if (k == Dimension - 1)
            i = First->Id;
    }
    N = FirstNode;
    do {
        if (!N->V)
            eprintf("TOUR_SECTION: Node is missing: %d", N->Id);
    } while ((N = N->Suc) != FirstNode);
    if (File == &SubproblemTourFile) {
        do {
            if (N->FixedTo1 &&
                N->SubproblemPred != N->FixedTo1
                && N->SubproblemSuc != N->FixedTo1)
                eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo1->Id);
            if (N->FixedTo2 && N->SubproblemPred != N->FixedTo2
                && N->SubproblemSuc != N->FixedTo2)
                eprintf("Fixed edge (%d, %d) "
                        "does not belong to subproblem tour", N->Id,
                        N->FixedTo2->Id);
        } while ((N = N->Suc) != FirstNode);
    }
    if (ProblemType == HPP)
        Dimension++;
    if (Asymmetric)
        Dimension *= 2;
    if (TraceLevel >= 1)
        printff("done\n");
}

static void Read_TYPE()
{
    unsigned int i;

    if (!(Type = Copy(strtok(0, Delimiters))))
        eprintf("TYPE: string expected");
    for (i = 0; i < strlen(Type); i++)
        Type[i] = (char) toupper(Type[i]);
    if (!strcmp(Type, "TSP"))
        ProblemType = TSP;
    else if (!strcmp(Type, "ATSP"))
        ProblemType = ATSP;
    else if (!strcmp(Type, "SOP"))
        ProblemType = SOP;
    else if (!strcmp(Type, "HCP"))
        ProblemType = HCP;
    else if (!strcmp(Type, "HPP"))
        ProblemType = HPP;
    else if (!strcmp(Type, "BWTSP"))
        ProblemType = BWTSP;
    else if (!strcmp(Type, "CCVRP"))
        ProblemType = CCVRP;
    else if (!strcmp(Type, "CVRP") || !strcmp(Type, "DCVRP"))
        ProblemType = CVRP;
    else if (!strcmp(Type, "ACVRP"))
        ProblemType = ACVRP;
    else if (!strcmp(Type, "CVRPTW"))
        ProblemType = CVRPTW;
    else if (!strcmp(Type, "MLP"))
        ProblemType = MLP;
    else if (!strcmp(Type, "OVRP"))
        ProblemType = OVRP;
    else if (!strcmp(Type, "PDPTW"))
        ProblemType = PDPTW;
    else if (!strcmp(Type, "PDTSP"))
        ProblemType = PDTSP;
    else if (!strcmp(Type, "PDTSPF") || !strcmp(Type, "PDTSPF"))
        ProblemType = PDTSPF;
    else if (!strcmp(Type, "PDTSPL") || !strcmp(Type, "PDTSPL"))
        ProblemType = PDTSPL;
    else if (!strcmp(Type, "TRP") || !strcmp(Type, "MTRP") ||
             !strcmp(Type, "MTRPD"))
        ProblemType = TRP;
    else if (!strcmp(Type, "RCTVRP"))
        ProblemType = RCTVRP;
    else if (!strcmp(Type, "RCTVRPTW"))
        ProblemType = RCTVRPTW;
    else if (!strcmp(Type, "STTSP"))
        ProblemType = STTSP;
    else if (!strcmp(Type, "TSPTW"))
        ProblemType = TSPTW;
    else if (!strcmp(Type, "VRPB"))
        ProblemType = VRPB;
    else if (!strcmp(Type, "VRPBTW"))
        ProblemType = VRPBTW;
    else if (!strcmp(Type, "VRPSPD") ||
             !strcmp(Type, "VRPSPDTW") ||
             !strcmp(Type, "VRPMPD") ||
             !strcmp(Type, "VRPMPDTW") || !strcmp(Type, "MVRPB"))
        ProblemType = VRPPD;
    else if (!strcmp(Type, "1-PDTSP"))
        ProblemType = ONE_PDTSP;
    else if (!strcmp(Type, "M-PDTSP"))
        ProblemType = M_PDTSP;
    else if (!strcmp(Type, "M1-PDTSP"))
        ProblemType = M1_PDTSP;
    else if (!strcmp(Type, "TSPDL"))
        ProblemType = TSPDL;
    else if (!strcmp(Type, "CTSP"))
        ProblemType = CTSP;
    else if (!strcmp(Type, "TOUR")) {
        ProblemType = TOUR;
        eprintf("TYPE: Type not implemented: %s", Type);
    } else
        eprintf("Unknown TYPE: %s", Type);
    Asymmetric =
        ProblemType == ATSP ||
        ProblemType == CCVRP ||
        ProblemType == ACVRP ||
        ProblemType == CVRPTW ||
        ProblemType == MLP ||
        ProblemType == M_PDTSP ||
        ProblemType == M1_PDTSP ||
        ProblemType == ONE_PDTSP ||
        ProblemType == OVRP ||
        ProblemType == PDTSP ||
        ProblemType == PDTSPF ||
        ProblemType == PDTSPL ||
        ProblemType == PDPTW ||
        ProblemType == RCTVRP ||
        ProblemType == RCTVRPTW ||
        ProblemType == SOP ||
        ProblemType == TRP ||
        ProblemType == TSPDL ||
        ProblemType == TSPTW ||
        ProblemType == VRPB ||
        ProblemType == VRPBTW || ProblemType == VRPPD;
}

static void Read_SERVICE_TIME()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%lf", &ServiceTime))
        eprintf("SERVICE_TIME: Real expected");
    if (ServiceTime < 0)
        eprintf("SERVICE_TIME: < 0");
}

static void Read_SERVICE_TIME_SECTION()
{
    int Id, i;
    Node *N;

    for (i = 1; i <= Dim; i++) {
        fscanint(ProblemFile, &Id);
        if (Id <= 0 || Id > Dim)
            eprintf("SERVICE_TIME_SECTION: Node number out of range: %d",
                    Id);
        N = &NodeSet[Id];
        if (!fscanf(ProblemFile, "%lf", &N->ServiceTime))
            eprintf("SERVICE_TIME_SECTION: "
                    "Missing service time for node %d", Id);
    }
}

/*
 The ReadTour function reads a tour from a file.
 
 The format is as follows:
 
 OPTIMUM = <real>
 Known optimal tour length. A run will be terminated as soon as a tour
 length less than or equal to optimum is achieved.
 Default: MINUS_INFINITY.
 
 TOUR_SECTION :
 A tour is specified in this section. The tour is given by a list of integers
 giving the sequence in which the nodes are visited in the tour. The tour is
 terminated by a -1.
 
 EOF
 Terminates the input data. The entry is optional.
 
 Other keywords in TSPLIB format may be included in the file, but they are
 ignored.
 */

void ReadTour(char *FileName, FILE ** File)
{
    char *Line, *Keyword, *Token;
    unsigned int i;
    int Done = 0;

    if (!(*File = fopen(FileName, "r")))
        eprintf("Cannot open tour file: \"%s\"", FileName);
    while ((Line = ReadLine(*File))) {
        if (!(Keyword = strtok(Line, Delimiters)))
            continue;
        for (i = 0; i < strlen(Keyword); i++)
            Keyword[i] = (char) toupper(Keyword[i]);
        if (!strcmp(Keyword, "COMMENT") ||
            !strcmp(Keyword, "DEMAND_SECTION") ||
            !strcmp(Keyword, "DEPOT_SECTION") ||
            !strcmp(Keyword, "DISPLAY_DATA_SECTION") ||
            !strcmp(Keyword, "DISPLAY_DATA_TYPE") ||
            !strcmp(Keyword, "EDGE_DATA_FORMAT") ||
            !strcmp(Keyword, "EDGE_DATA_SECTION") ||
            !strcmp(Keyword, "EDGE_WEIGHT_FORMAT") ||
            !strcmp(Keyword, "EDGE_WEIGHT_SECTION") ||
            !strcmp(Keyword, "EDGE_WEIGHT_TYPE") ||
            !strcmp(Keyword, "FIXED_EDGES_SECTION") ||
            !strcmp(Keyword, "NAME") ||
            !strcmp(Keyword, "NODE_COORD_SECTION") ||
            !strcmp(Keyword, "NODE_COORD_TYPE")
            || !strcmp(Keyword, "TYPE"));
        else if (strcmp(Keyword, "OPTIMUM") == 0) {
            if (!(Token = strtok(0, Delimiters)) ||
                !sscanf(Token, GainInputFormat, &Optimum))
                eprintf("[%s] (OPTIMUM): Integer expected", FileName);
        } else if (strcmp(Keyword, "DIMENSION") == 0) {
            int Dim = 0;
            if (!(Token = strtok(0, Delimiters)) ||
                !sscanf(Token, "%d", &Dim))
                eprintf("[%s] (DIMENSION): Integer expected", FileName);
            if (Dim != DimensionSaved && Dim != Dimension) {
                printff("Dim = %d, DimensionSaved = %d, Dimension = %d\n",
                        Dim, DimensionSaved, Dimension);
                eprintf
                    ("[%s] (DIMENSION): does not match problem dimension",
                     FileName);
            }
        } else if (!strcmp(Keyword, "PICKUP_AND_DELIVERY_SECTION")) {
            Read_PICKUP_AND_DELIVERY_SECTION();
        } else if (!strcmp(Keyword, "SERVICE_TIME_SECTION")) {
            Read_SERVICE_TIME_SECTION();
        } else if (!strcmp(Keyword, "TOUR_SECTION")) {
            Read_TOUR_SECTION(File);
            Done = 1;
        } else if (!strcmp(Keyword, "EOF"))
            break;
        else
            eprintf("[%s] Unknown Keyword: %s", FileName, Keyword);
    }
    if (!Done)
        eprintf("Missing TOUR_SECTION in tour file: \"%s\"", FileName);
    fclose(*File);
}

static void Read_RISK_THRESHOLD()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &RiskThreshold))
        eprintf("RISK_THRESHOLD: Integer expected");
}

static void Read_SALESMEN()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || (Salesmen == 1 && !sscanf(Token, "%d", &Salesmen)))
        eprintf("SALESMEN/VEHICLES: Integer expected");
    if (Salesmen <= 0)
        eprintf("SALESMEN/VEHICLES: <= 0");
}

static void Read_SCALE()
{
    char *Token = strtok(0, Delimiters);

    if (!Token || !sscanf(Token, "%d", &Scale))
        eprintf("SCALE: Integer expected");
    if (Scale < 1)
        eprintf("SCALE: < 1");
}

static void Convert2FullMatrix()
{
    int n = DimensionSaved, i, j;
    Node *Ni, *Nj;

    if (Scale < 1)
        Scale = 1;
    if (n > MaxMatrixDimension) {
        OldDistance = Distance;
        Distance = Distance_Asymmetric;
        for (i = 1; i <= n; i++) {
            Ni = &NodeSet[i];
            Nj = &NodeSet[i + n];
            Nj->X = Ni->X;
            Nj->Y = Ni->Y;
            Nj->Z = Ni->Z;
            FixEdge(Ni, Nj);
        }
        return;
    }
    CostMatrix = (int *) calloc((size_t) n * n, sizeof(int));
    for (i = 1; i <= n; i++) {
        Ni = &NodeSet[i];
        Ni->C = &CostMatrix[(size_t) (i - 1) * n] - 1;
    }
    for (i = 1; i <= Dim; i++) {
        Ni = &NodeSet[i];
        for (j = i + 1; j <= Dim; j++) {
            Nj = &NodeSet[j];
            Ni->C[j] = Nj->C[i] = Distance(Ni, Nj);
        }
    }
    for (i = 1; i <= n; i++)
        FixEdge(&NodeSet[i], &NodeSet[i + n]);
    c = 0;
    Distance = Distance_ATSP;
    WeightType = -1;
}
