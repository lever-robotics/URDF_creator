import { useState, useRef } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import MenuModal from '../../FunctionalComponents/MenuModal';

const MenuIcon = ({ openProjectManager, openImportDisplayer, openExportDisplayer }) => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const buttonRef = useRef(null);

    const menuItems = [
        { label: "Project Manager", action: () => {
            handleClick();
            openProjectManager();
        } },
        { label: "Import", action: () => {
            handleClick();
            openImportDisplayer();
        } },
        { label: "Export", action: () => {
            handleClick();
            openExportDisplayer();
        } },
        // { label: "Settings", action: () => alert("Setting") }
    ];

    const handleClick = () => {
        if(isModalOpen){
            setIsModalOpen(false);
        }else{
            setIsModalOpen(true);
        }
    };

    return (
        <>
            <button className="menu-icon-button" onClick={handleClick} ref={buttonRef}>
                <FontAwesomeIcon icon={faBars} />
            </button>
            <MenuModal
            isOpen={isModalOpen}
            onClose={handleClick}
            menuItems={menuItems}
            buttonRef={buttonRef}
            />
        </>
    );
};

export default MenuIcon;