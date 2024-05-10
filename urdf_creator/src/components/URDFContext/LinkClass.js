// Link.js
export class Link {
    constructor(value, {
        position = { x: 0, y: 0, z: 0 },
        orientation = { roll: 0, pitch: 0, yaw: 0 },
        isBaseLink = false,
        size = { width: 1, height: 1, depth: 1 },
        joint = {
            parentFrameId: '',
            selfFrameId: '',
            jointType: 'fixed'
        }
    } = {}) {
        this.value = value;
        this.position = position;
        this.orientation = orientation;
        this.isBaseLink = isBaseLink;
        this.size = size;
        this.joint = joint;
        this.children = [];
    }
}