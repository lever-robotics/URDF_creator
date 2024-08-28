import React from "react";
import "./MenuModal.css";
import MenuItem from "./MenuItem";

const MenuModal = ({ isOpen, onClose, menuItems, buttonRef }) => {
    if (!isOpen) return null;

    const buttonRect = buttonRef.current.getBoundingClientRect();
    const position = {
        top: buttonRect.bottom + window.scrollY, // Position below button
        left: buttonRect.left + window.scrollX, // Align left with the button
    }

    return (
        <div
            className="menu-modal"
            style={{ top: position.top , left: position.left }}>
            <button className="close-button" onClick={onClose}>
                &times;
            </button>
            <ul className="menu-list">
                {menuItems.map((item, index) => (
                    <MenuItem index={index} action={item.action} label={item.label}/>
                ))}
            </ul>
        </div>
    );
};

export default MenuModal;
