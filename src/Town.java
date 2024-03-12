import java.util.*;

public class Town {

    public String townName;
    public LinkedList<String> adjTowns;
    public double lat;
    public double lon;
    public double f;
    public double g;
    public double h;
    public String parent;

    Town(String town, double latitude, double longitude) {
        townName = town;
        lat = latitude;
        lon = longitude;
        adjTowns = new LinkedList<String>();
    }

    //inserts adjacent town into list
    void addEdge(String town) {
        if (!adjTowns.contains(town)) {
            adjTowns.add(town);
        }
    }

}
