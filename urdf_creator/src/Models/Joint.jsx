export default class Joint {
    constructor(type = "fixed", axis = [1, 0, 0], parentLink = null, childLink = null, origin = [0, 0, 0], name = "") {
        this.type = type;
        this.axis = axis;
        this.parentLink = parentLink;
        this.childLink = childLink;
        this.origin = origin;
        this.name = name;
    }
}