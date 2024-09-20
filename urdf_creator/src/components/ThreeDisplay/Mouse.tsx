import React from "react"

export class Mouse {

    mountRef: React.MutableRefObject<HTMLDivElement | null>;
    eventFunctions: { type: string; func: (event: any) => void; }[];
    previousUpTime: number;
    currentDownTime: number;
    startPos: [number, number];
    onClickFunctions: Function;
    onDoubleClickFunctions: Function[];
    x: number;
    y: number;

    constructor(mountRef: React.MutableRefObject<HTMLDivElement | null>) {
        this.mountRef = mountRef;
        this.eventFunctions = [
            { type: "pointerdown", func: this.onMouseDown.bind(this) },
            { type: "pointerup", func: this.onMouseUp.bind(this) },
        ];
        this.previousUpTime = 0;
        this.currentDownTime = 0;
        this.startPos = [0, 0];

        this.onClickFunctions = ()=> {};
        this.onDoubleClickFunctions = [];
        this.x = 0;
        this.y = 0;
    }

    addListeners() {
        if (this.mountRef.current) {
            this.eventFunctions.forEach((event) => {
                this.mountRef.current!.addEventListener(event.type, event.func);
            });
        }
    }

    callback() {
        if (this) {
            if (this.mountRef.current) {
                this.eventFunctions.forEach((event) => {
                    this.mountRef.current!.removeEventListener(event.type, event.func);
                });
            }
        }
    }

    onMouseDown(event: React.MouseEvent<HTMLDivElement>) {
        if ((event.target as HTMLElement).localName !== "canvas") return;
        
        event.preventDefault();
        this.currentDownTime = Date.now();
        this.startPos = [event.clientX, event.clientY];
    }

    onMouseUp(event: React.MouseEvent<HTMLDivElement>) {
        if ((event.target as HTMLElement).localName !== "canvas") return;
        
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

    onDoubleClick(event: React.MouseEvent<HTMLDivElement>) {
        this.onDoubleClickFunctions.forEach((func) => {
            func(event);
        });
    }

    addOnDoubleClickFunctions(func: (event: MouseEvent) => void) {
        this.onDoubleClickFunctions.push(func);
    }

    addOnClickFunctions(func: (event: MouseEvent) => void) {
        this.onClickFunctions = func;
    }

    onClick(event: React.MouseEvent<HTMLDivElement>) {
        this.onClickFunctions(event)
}
}

// in react i have a function in one component A. This component has a ref to another component B, and there is a variable foo in B that A sets to a function bar. When B calls foo, it is calling bar in A. Bar reads a state of A, when A calls bar it reads the state correctly. But when B calls foo, it reads the state as it was when foo was set to bar. How can I fix this? 