import React from "react";
import { useRef } from "react";
import "./MenuModal.css";
import MenuItem from "./MenuItem";
import useClickOutside from "./useClickOutside";

type Props = {
    onClose: () => void,
    menuItems: {
        label: string;
        action: () => void;
    }[],
    buttonRef: React.RefObject<HTMLButtonElement>,
}

const MenuModal: React.FC<Props> = ({ onClose, menuItems, buttonRef }) => {
    
    const wrapperRef = useRef<HTMLDivElement>(null);
    useClickOutside(wrapperRef, onClose);

    const buttonRect = buttonRef.current!.getBoundingClientRect();
    const position = {
        top: buttonRect.bottom + window.scrollY, // Position below button
        left: buttonRect.left + window.scrollX, // Align left with the button
    }

    return (
        <div
            className="menu-modal"
            ref={wrapperRef}
            style={{ top: position.top , left: position.left }}>
            <ul className="menu-list">
                {menuItems.map((item, index) => (
                    <MenuItem key={index} action={item.action} label={item.label}/>
                ))}
            </ul>
        </div>
    );
};

export default MenuModal;
