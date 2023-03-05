package frc.lib.util;

public enum LEDColor {
    PURPLE(70, 2, 115),
    YELLOW(150, 131, 2),
    RED(255, 0, 0),
    BLUE(0, 0, 255),
    GREEN(0, 255, 0),
    WHITE(255, 255, 255),
    CYAN(0, 255, 255),
    ORANGE(252, 144, 3),
    OFF(0, 0, 0); 

    public int r;
    public int g;
    public int b;

    private LEDColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }
}
