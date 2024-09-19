import Inertia from "./Inertia";

export default class UserData {
    constructor(shape, name) {
        this.name = name;
        this.shape = shape;
        this.inertia = new Inertia();
        this.isRootFrame = false;
        this.selectable = true;
        this.sensor = null; // Sensor object
        this.stlfile = null;
        this.material = null;
        this.color = null;
    }

    duplicate() {
        const duplicated = new UserData(this.shape);
        duplicated.stlfile = this.stlfile;
        duplicated.name = this.name + " copy";
        duplicated.inertia = this.inertia.duplicate();
        duplicated.isRootFrame = false;
        if (this.sensor) {
            duplicated.sensor = this.sensor.duplicate();
        }

        return duplicated;
    }
}
