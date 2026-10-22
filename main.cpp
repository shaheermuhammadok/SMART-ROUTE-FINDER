
#include "raylib.h"     // Graphic Library (Text,2 Vector,Color,Inite Window)
#include <vector>       // Used For Dynamic (For Resizeing it self Automatically)
#include <queue>        // Used as Fifo and (Used as Mini Heap)
#include <string>       // used for the Handelation of Lacation name
#include <algorithm>    // PreDefine Function (Alternative Route, Reversing path)
#include <cmath>        // Math Function (Distance Calculator,)
#include <limits>       // Are boundaries help us Before overflowing the data 
#include <cstdio>       // Wrapped the C Library 

using namespace std;

const float INF = numeric_limits<float>::infinity();// Inf Special float value that is similar to infinity
const int MAP_W = 900, WIN_W = 1220, WIN_H = 720; // Giving the Dimenstion for the Window

//  GRAPH YE User Define Data type Ha for Trafic Conditions
enum Cond { CLEAR = 0, TRAFFIC, CONSTRUCTION, CLOSED };  
                                               
struct Node { string name; Vector2 pos; };
struct Road { int a, b; float km; Cond cond; };
struct Edge { int to; int roadId; };          

class Graph {
public:
    vector<Node> nodes;
    vector<Road> roads;
    vector<vector<Edge>> adj;                 // ADJACENCY LIST

    void addNode(const string& n, float x, float y) {
        nodes.push_back({n, {x, y}});
        adj.emplace_back();
    }
    void addRoad(int a, int b) {            //To Add Road between two nodes/ Location
        float dx = nodes[a].pos.x - nodes[b].pos.x, dy = nodes[a].pos.y - nodes[b].pos.y;
        float km = sqrtf(dx * dx + dy * dy) / 30.0f;      
        int id = (int)roads.size();         // Position and Exits
        roads.push_back({a, b, km, CLEAR});
        adj[a].push_back({b, id});              // Informiing about the surrunding node about its position and and which road to take
        adj[b].push_back({a, id});                    
    }
    // Use to Calculate Time Two travel from One Path to another Path
    float cost(int roadId) const {
        const Road& r = roads[roadId];
        switch (r.cond) {
            case CLEAR:        return r.km * 1.0f;
            case TRAFFIC:      return r.km * 2.2f;
            case CONSTRUCTION: return r.km * 3.5f;
            case CLOSED:       return INF;               
        }
        return r.km;
    }
};

struct Route {
    vector<int> nodes;      // Location
    vector<int> roads;      // Roads
    float cost = INF;       // Total Time it will Take Firstly it's impossible 
    float km = 0;           // 
    bool valid = false;
};

//  ALGORITHM 1: DIJKSTRA 7

Route dijkstra(const Graph& g, int src, int dst, vector<int>& settleOrder) {
    int n = (int)g.nodes.size();    // How many Way reach to that path or Exits
    vector<float> dist(n, INF);     // The best Time to reach the place
    vector<int> prevNode(n, -1), prevRoad(n, -1); // The Best Route the Reach the place
    vector<bool> done(n, false);    // When the Best path is Selected with best time 
    settleOrder.clear();            // Make Sure the list start Empty with 

    using P = pair<float, int>;                           // P = Dis,Tim
    priority_queue<P, vector<P>, greater<P>> pq;          // MIN-HEAP Alway give the Shortest Time First 
    dist[src] = 0;                                        // Distance from Start when its Zero
    pq.push({0, src});                                    // Put at stating place and Time =Zero

    while (!pq.empty()) {           // Will go until the Heap is Empty
        auto [d, u] = pq.top(); pq.pop();  // Shortest Time and Distance to the path d=time, u=place
        if (done[u]) continue;             // If Find this place before we will not find the route again
        done[u] = true;                    // Marked the place that we have find the distance
        settleOrder.push_back(u);          // Remember the Command in which the place is Finallized
        if (u == dst) break;               // Confirm the Place then Stop the Algorithms
        for (const Edge& e : g.adj[u]) {   //  neighbours Finallized Place to cheak if the Neighbour is Finallized
            float w = g.cost(e.roadId);    // Travel Time to that neighbour
            if (w == INF) continue;        // Stop if the Neighbour place is Blocked
            if (dist[u] + w < dist[e.to]) { // The best path to SURRONDING Location
                dist[e.to] = dist[u] + w;   // The best know time
                prevNode[e.to] = u;         // Remember we reach the place
                prevRoad[e.to] = e.roadId;  //Remeber the path that we took
                pq.push({dist[e.to], e.to});//Put surronding to heap to reconized it later
            }
        }
    }

    Route r;    // The route will build and then it will return
    if (dist[dst] == INF) return r;                       // unreachable (if it still Infinity)
    // Rebuild path by look the Neighbour Path THE PREVIOUS pATH
    for (int v = dst; v != -1; v = prevNode[v]) { // Start at destination and how we got there
        r.nodes.push_back(v);   // Add this place to the route FROM  End Place to Start
        if (prevRoad[v] != -1) { r.roads.push_back(prevRoad[v]); r.km += g.roads[prevRoad[v]].km; }
    } // If there wa a route used to reash there Add it to the totall route 
    reverse(r.nodes.begin(), r.nodes.end());// Flip the List So ti Could Read from Starting to End
    reverse(r.roads.begin(), r.roads.end());// Flip the Distance so it could tell path from biggening to end
    r.cost = dist[dst];     // Store the Total Time
    r.valid = true;         // Marked the Tools as unable to use
    return r;               // return the route
}

//  ALGORITHM 2: DFS + BACKTRACKING 

void dfsAllPaths(const Graph& g, int u, int dst, vector<bool>& visited,         //u=Sart/currentPlace,dst=endPlace,visited path 
                 vector<int>& pathN, vector<int>& pathR,                        // Visited that we visited often
                 float cost, float km, float bound, vector<Route>& out) {       // The path so far the path we have to cover
    if (out.size() >= 20000) return;    // Stop if we huge number of path
    if (u == dst) {                     // If we Reash the Destination
        Route r; r.nodes = pathN; r.roads = pathR; r.cost = cost; r.km = km; r.valid = true;// add route to package as Finish, Valid Route
        out.push_back(r); //Finished this route into the result 
        return;
    }
    if (pathN.size() >= 10) return;                       // limit path length Give Up
    for (const Edge& e : g.adj[u]) {                       // Try Every place of the neighbour we are currently on 
        if (visited[e.to]) continue;                       // skip we are already on the path
        float w = g.cost(e.roadId);                        // How much cost to the neighbour
        if (w == INF || cost + w > bound) continue;       // closed or too expensive
        visited[e.to] = true;                             // Marked Vusuted 
        pathN.push_back(e.to); pathR.push_back(e.roadId); // Add to Current path
        dfsAllPaths(g, e.to, dst, visited, pathN, pathR, cost + w, km + g.roads[e.roadId].km, bound, out); // Exploring new places
        pathN.pop_back(); pathR.pop_back();               // un-choose (BACKTRACK) Ditching the neighbour that path
        visited[e.to] = false;                          // unvisited Path
    }
}

// Best route (Dijkstra) + up to 3 alternatives (DFS, sorted by cost).
vector<Route> findRoutes(const Graph& g, int src, int dst, vector<int>& settleOrder) {
    vector<Route> result;       // Now the Finall route path will be return
    Route best = dijkstra(g, src, dst, settleOrder);    // Find the Single best route using 
    if (!best.valid) return result;         //no route empty
    result.push_back(best);                 // the best route become result one

    vector<Route> all;                  //ALL ALTERNATIVE ROUTE Find by DFS manager
    vector<bool> vis(g.nodes.size(), false); //Tracking of Visisted path by DFS search
    vector<int> pn{src}, pr;        //Path so far starting with the starting path
    vis[src] = true;                //Marked start as visited
    dfsAllPaths(g, src, dst, vis, pn, pr, 0, 0, best.cost * 1.8f, all);
    //run the dfs path  up to 1.8 cost 
    sort(all.begin(), all.end(), [](const Route& a, const Route& b) { return a.cost < b.cost; }); // Sort all the path for cheap to Expensive
    for (const Route& r : all) {  // Going through all algorithms as listing all the pth
        if (result.size() >= 4) break;
        if (r.nodes != best.nodes) result.push_back(r);
    }
    return result;
}

//  UI HELPERS 
float distPointSegment(Vector2 p, Vector2 a, Vector2 b) {
    float dx = b.x - a.x, dy = b.y - a.y;   // Direction of the road line
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);  // How far line closet poit is
    t = fmaxf(0, fminf(1, t)); //to make the pointer keep on road 
    float cx = a.x + t * dx, cy = a.y + t * dy; // Theactual Closess point to the road
    return sqrtf((p.x - cx) * (p.x - cx) + (p.y - cy) * (p.y - cy)); // Returnt the distance between the point
}

Color condColor(Cond c) {
    switch (c) {
        case TRAFFIC:      return RED;
        case CONSTRUCTION: return ORANGE;
        case CLOSED:       return DARKGRAY;
        default:           return Color{170, 175, 185, 255};  // Cordinate for Grey Color
    }
}
const char* condName(Cond c) { //Turn Const into a readable Format
    static const char* n[] = {"Clear", "Heavy traffic", "Construction", "CLOSED"}; //list of label in the same ordered
    return n[c];    //return the label in the same mathcing condition
}

void drawRouteLine(const Graph& g, const Route& r, Color col, float thick) {
    for (size_t i = 0; i + 1 < r.nodes.size(); i++) //Drawing the route on the Screen by connecting it line 
        DrawLineEx(g.nodes[r.nodes[i]].pos, g.nodes[r.nodes[i + 1]].pos, thick, col);
}   // Draw the Line Next to Place 

// Position of a moving car along a route (t in [0,1)).
Vector2 pointOnRoute(const Graph& g, const Route& r, float t) {
    float total = 0;        //Route Lenght
    vector<float> seg;      //devide route lenght into different part
    for (size_t i = 0; i + 1 < r.nodes.size(); i++) {//Measure pth every route
        Vector2 a = g.nodes[r.nodes[i]].pos, b = g.nodes[r.nodes[i + 1]].pos;//Two end od Segment or Diided Route
        float l = sqrtf((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));//Lenght of Divided path is equAl to one
        seg.push_back(l); total += l;//cheaking if Divided path is Equal to Full Path
    }
    float target = t * total; //how far the car have to travel
    for (size_t i = 0; i < seg.size(); i++) {   // Travelling with car 
        if (target <= seg[i]) {     //if the car following the path 
            Vector2 a = g.nodes[r.nodes[i]].pos, b = g.nodes[r.nodes[i + 1]].pos;
            float k = target / seg[i];      //How far along this Segments
            return {a.x + (b.x - a.x) * k, a.y + (b.y - a.y) * k};  //Return Exact position of the Car 
        }
        target -= seg[i];   // When Move Subrat the Segment and Subtract the Next one 
    }
    return g.nodes[r.nodes.back()].pos;  // return to the previous position of the car 
}

//  MAIN 
int main() {
    Graph g;
    // Places (id = order of insertion)
    g.addNode("Airport", 80, 100);     // 0 -16
    g.addNode("Saddar", 230, 80);      
    g.addNode("University", 400, 110); 
    g.addNode("Hospital", 600, 80);    
    g.addNode("Stadium", 800, 130);    
    g.addNode("Market", 150, 260);     
    g.addNode("Central Park", 330, 270); 
    g.addNode("Mall", 520, 250);       
    g.addNode("Station", 720, 290);    
    g.addNode("Old Town", 90, 430);    
    g.addNode("Library", 270, 450);    
    g.addNode("Bus Depot", 450, 420);  
    g.addNode("Tech Park", 650, 440);  
    g.addNode("Harbor", 820, 470);     
    g.addNode("Suburb", 200, 620);     
    g.addNode("Downtown", 420, 600);   
    g.addNode("Beach", 700, 620);      

    int links[][2] = {          //List which node should be conneted to each other
        {0,1},{1,2},{2,3},{3,4},{0,5},{1,5},{1,6},{2,6},{2,7},{3,7},{4,8},{5,6},
        {6,7},{7,8},{5,9},{6,10},{6,11},{7,11},{7,12},{8,12},{8,13},{9,10},{10,11},
        {11,12},{12,13},{9,14},{10,14},{10,15},{11,15},{12,16},{13,16},{14,15},{15,16}};
    for (auto& l : links) g.addRoad(l[0], l[1]);            // Build Road between two nodes

    InitWindow(WIN_W, WIN_H, "Smart Route  Finder - DSA Project");
    SetTargetFPS(60);

    int startNode = -1, endNode = -1, selected = 0;
    vector<Route> routes;       //no route selected
    vector<int> settleOrder;    // no previous route
    Color routeCol[4] = {Color{0, 110, 255, 255}, Color{160, 60, 200, 255},     //Show Different Path on screen With Different  color
                         Color{0, 160, 140, 255}, Color{200, 120, 0, 255}};

    auto recompute = [&]() {            //Function to refresh route 
        routes.clear(); settleOrder.clear(); selected = 0;      // Wipe every thing out
        if (startNode >= 0 && endNode >= 0) routes = findRoutes(g, startNode, endNode, settleOrder);
    };      // Actually Find roue based on it start and end point

    while (!WindowShouldClose()) {          //Repeat every Thing Untill the Keep window open
        Vector2 m = GetMousePosition();     // Get Position of the Mouse

        // ---- input ----
        if (m.x < MAP_W) {      //Only handle click when the mouse is on the map
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {      //Left Click only
                for (int i = 0; i < (int)g.nodes.size(); i++) {
                    Vector2 p = g.nodes[i].pos;     //Cheak on Every Position it was Click and if the position is on Map then register it 
                    if (sqrtf((m.x - p.x) * (m.x - p.x) + (m.y - p.y) * (m.y - p.y)) < 18) {
                        if (startNode == -1 || (startNode != -1 && endNode != -1)) { startNode = i; endNode = -1; routes.clear(); settleOrder.clear(); }
                        else if (i != startNode) endNode = i;       //when click cheak wheater its click before one or twice 
                        recompute();
                        break;
                    }
                }
            }
            if (IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) {     //cheaking for right click
                for (Road& r : g.roads) {       //cheaking for Right click on any road 
                    if (distPointSegment(m, g.nodes[r.a].pos, g.nodes[r.b].pos) < 8) {  //If the Mouse is Within 8 pixal Click it 
                        r.cond = (Cond)((r.cond + 1) % 4);      // Tell the road Condition on that road 
                        recompute(); //refresh the route with new selection 
                        break;      //Stop cheaking for other places
                    }
                }
            }
        }
        if (IsKeyPressed(KEY_C)) { startNode = endNode = -1; routes.clear(); settleOrder.clear(); } //Clear the Starting And Ending Point
        if (IsKeyPressed(KEY_R)) { for (Road& r : g.roads) r.cond = CLEAR; recompute(); } // cLEAR THE road 
        for (int k = 0; k < 4; k++) // Cheak if any number is pressed
            if (IsKeyPressed(KEY_ONE + k) && k < (int)routes.size()) selected = k;  //k if that many exits  switch to highlited Route

        //  draw 
        BeginDrawing();
        ClearBackground(Color{238, 241, 246, 255});//Gray Color 

        // roads
        for (const Road& r : g.roads) {         // Draw the Road on the map
            Vector2 a = g.nodes[r.a].pos, b = g.nodes[r.b].pos;    //Starting and Ending point of the routes 
            DrawLineEx(a, b, r.cond == CLEAR ? 4.0f : 6.0f, condColor(r.cond)); // Draw ROAD Line Thicker if any problem
            Vector2 mid = {(a.x + b.x) / 2, (a.y + b.y) / 2}; // Select midpoint of the road for label
            if (r.cond == CLOSED) DrawText("X", (int)mid.x - 5, (int)mid.y - 9, 18, MAROON); // 
            else if (r.cond != CLEAR) DrawText("!", (int)mid.x - 3, (int)mid.y - 9, 18, WHITE);
            DrawText(TextFormat("%.1f", r.km), (int)mid.x + 6, (int)mid.y + 4, 12, Color{90, 90, 100, 255});// Draw lenght One Decimal
        }

        // routes (alternatives thin, selected thick)
        for (int i = (int)routes.size() - 1; i >= 0; i--)   // Go throught Every and Draw Every Route Faded (!Selected)
            if (i != selected) drawRouteLine(g, routes[i], Fade(routeCol[i], 0.45f), 5); 
        if (!routes.empty()) {      //Drawing the Selected Route 
            drawRouteLine(g, routes[selected], routeCol[selected], 9);
            float t = fmodf((float)GetTime() * 0.15f, 1.0f);          // animated car
            Vector2 car = pointOnRoute(g, routes[selected], t);
            DrawCircleV(car, 8, WHITE);         //Color Car White and the outline Black
            DrawCircleLinesV(car, 8, BLACK);
        }

        // places
        for (int i = 0; i < (int)g.nodes.size(); i++) {         //Drawing Every Place on the Map Dark Grey
            Color c = Color{60, 70, 90, 255};
            if (i == startNode) c = Color{0, 170, 60, 255};     // Strting point Green
            if (i == endNode) c = Color{220, 30, 40, 255};      // Ending Point Read
            DrawCircleV(g.nodes[i].pos, 15, c);
            DrawCircleLinesV(g.nodes[i].pos, 15, WHITE);
            int w = MeasureText(g.nodes[i].name.c_str(), 15);   //How wide the place width and Text Should Be
            DrawText(g.nodes[i].name.c_str(), (int)g.nodes[i].pos.x - w / 2, (int)g.nodes[i].pos.y + 20, 15, BLACK);
            // Dijkstra settle order (shows the min-heap order of exploration)
            for (int k = 0; k < (int)settleOrder.size(); k++) // Cheak if this Place appear 
                if (settleOrder[k] == i) {      
                    const char* s = TextFormat("%d", k + 1);        // If visited Order text to show One,two,three
                    DrawText(s, (int)g.nodes[i].pos.x - MeasureText(s, 14) / 2, (int)g.nodes[i].pos.y - 7, 14, WHITE);
                }   // draw at center top of circle
        }

        // side panel
        DrawRectangle(MAP_W, 0, WIN_W - MAP_W, WIN_H, Color{28, 32, 44, 255}); //DRAWING THE SIDE Panel
        int x = MAP_W + 16, y = 14;// tell its position 
        DrawText("SMART ROUTE FINDER", x, y, 20, WHITE); y += 34;
        DrawText("Left-click: start, then end", x, y, 14, LIGHTGRAY); y += 18; 
        DrawText("Right-click road: change condition", x, y, 14, LIGHTGRAY); y += 18;
        DrawText("Keys 1-4: choose route  C/R: reset", x, y, 14, LIGHTGRAY); y += 30;

        DrawText(TextFormat("Start: %s", startNode >= 0 ? g.nodes[startNode].name.c_str() : "-"), x, y, 16, GREEN); y += 22;
        DrawText(TextFormat("End:   %s", endNode >= 0 ? g.nodes[endNode].name.c_str() : "-"), x, y, 16, Color{255, 110, 110, 255}); y += 34;

        if (startNode >= 0 && endNode >= 0 && routes.empty()) {
            DrawText("No route available!", x, y, 18, RED); y += 24;
            DrawText("All paths are closed.", x, y, 14, LIGHTGRAY);
        }
        for (int i = 0; i < (int)routes.size(); i++) {
            DrawRectangle(x - 6, y - 4, WIN_W - MAP_W - 20, 26, i == selected ? Color{60, 68, 92, 255} : Color{40, 45, 62, 255});
            DrawRectangle(x - 6, y - 4, 6, 26, routeCol[i]); // Draw BrightBox if the route is Selected
            DrawText(TextFormat("%d. %s  %.1f min  %.1f km", i + 1, i == 0 ? "BEST" : "Alt ", routes[i].cost, routes[i].km),
                     x + 8, y, 16, WHITE); // Draw Route Rank Best to the Worst Alternative
            y += 32;
        }
        if (!routes.empty()) {
            y += 8;
            DrawText("Selected path:", x, y, 15, LIGHTGRAY); y += 22;
            const Route& r = routes[selected];  // Section Heading
            for (size_t i = 0; i < r.nodes.size(); i++) {   // List the Every Spot on that route
                DrawText(TextFormat("%d) %s", (int)i + 1, g.nodes[r.nodes[i]].name.c_str()), x + 6, y, 15, WHITE);
                y += 18;
            }
            y += 10;
            DrawText("Road problems on this route:", x, y, 15, LIGHTGRAY); y += 20; //Section Heading Wheather We found any problem on the road
            bool any = false;
            for (int rid : r.roads)
                if (g.roads[rid].cond != CLEAR) {
                    any = true;
                    DrawText(TextFormat("%s - %s: %s", g.nodes[g.roads[rid].a].name.c_str(),
                             g.nodes[g.roads[rid].b].name.c_str(), condName(g.roads[rid].cond)), x + 6, y, 13, ORANGE);
                    y += 16;
                }
            if (!any) DrawText("None - clear roads", x + 6, y, 14, GREEN);
        }

        // legend
        int ly = WIN_H - 88;    //Mentioning Traffic Status Grey,Red, Orange at the End of the page and corner
        DrawRectangle(x, ly, 18, 6, Color{170, 175, 185, 255});   DrawText("Clear (x1.0)", x + 26, ly - 5, 14, WHITE);
        DrawRectangle(x, ly + 20, 18, 6, RED);                   DrawText("Heavy traffic (x2.2)", x + 26, ly + 15, 14, WHITE);
        DrawRectangle(x, ly + 40, 18, 6, ORANGE);                DrawText("Construction (x3.5)", x + 26, ly + 35, 14, WHITE);
        DrawRectangle(x, ly + 60, 18, 6, DARKGRAY);              DrawText("Closed (blocked)", x + 26, ly + 55, 14, WHITE);

        EndDrawing();   // End Drawing
    }   //Close Loop
    CloseWindow(); // Close the windows
    return 0;
}
