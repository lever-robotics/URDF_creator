export class Mouse {
    constructor(mountRef) {
        this.mountRef = mountRef;
        this.eventFunctions = [
            { type: "pointerdown", func: this.onMouseDown.bind(this) },
            { type: "pointerup", func: this.onMouseUp.bind(this) },
        ];
        this.previousUpTime = null;
        this.currentDownTime = null;
        this.startPos = null;

        this.onClickFunctions = [];
        this.onDoubleClickFunctions = [];
    }

    addListeners() {
        if (this.mountRef) {
            this.eventFunctions.forEach((event) => {
                this.mountRef.addEventListener(event.type, event.func);
            });
        }
    }

    callback() {
        if (this) {
            if (this.mountRef) {
                this.eventFunctions.forEach((event) => {
                    this.mountRef.removeEventListener(event.type, event.func);
                });
            }
        }
    }

    onMouseDown(event) {
        if (event.target.localName !== "canvas") return;
        event.preventDefault();
        this.currentDownTime = Date.now();
        this.startPos = [event.clientX, event.clientY];
    }

    onMouseUp(event) {
        if (event.target.localName !== "canvas") return;
        event.preventDefault();
        const clickTime = 300;
        const dragThreshold = 20;
        const endPos = [event.clientX, event.clientY];

        // if the user clicks off of the three scene then drags in, return
        if (!this.startPos) return;

        if (Math.sqrt((endPos[0] - this.startPos[0]) ** 2 + (endPos[1] - this.startPos[1]) ** 2) > dragThreshold) {
            // Do nothing if dragged
        } else if (this.currentDownTime - this.previousUpTime < clickTime && Date.now() - this.currentDownTime < clickTime) {
            this.onDoubleClick(event);
        } else if (Date.now() - this.currentDownTime < clickTime) {
            this.onClick(event);
        }
        this.previousUpTime = Date.now();
    }

    onDoubleClick(event) {
        this.onDoubleClickFunctions.forEach((func) => {
            func(event);
        });
    }

    addOnDoubleClickFunctions(func) {
        this.onDoubleClickFunctions.push(func);
    }

    addOnClickFunctions(func) {
        this.onClickFunctions.push(func);
    }

    onClick(event) {
        this.onClickFunctions.forEach((func) => {
            func(event);
        });
    }
}
