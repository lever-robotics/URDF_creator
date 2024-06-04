import * as React from 'react';
import { useState } from 'react';
import Button from '@mui/material/Button';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import { styled } from '@mui/material/styles';
import Typography from '@mui/material/Typography';
import PopupState, { bindTrigger, bindMenu } from 'material-ui-popup-state';
import './MenuModal.css'

export default function MenuModal({ openProjectManager }) {
  const [projectTitle, setProjectTitle] = useState('untitled');

  const StyledMenuItem = styled(props => (
    <MenuItem {...props} disableRipple>
    </MenuItem>
  ))(({ theme }) => ({
    minWidth: 0,
    fontSize: "3em",
    fontWeight: 200,
    fontFamily: "inherit",
    transition: 'font-size 0.4s',
    '&:hover, &.Mui-focusVisible':{
      fontSize: '4em',
    },
  }));

  const StyledMenu = styled((props) => (
    <Menu
      {...props}
    />
  ))(({ theme }) => ({
    '& .MuiPaper-root': {
      background: 'transparent',
      color: 'white',
      boxShadow: 'none',
      height: '100%',
      width: '30%',
    },
    '& .MuiList-root': {
      textOverflow: 'ellipsis',
      height: '90%',
      display: 'flex',
      paddingLeft: '15%',
      justifyContent: 'space-evenly',
      flexDirection: 'column',
    },
    background:'rgba(0, 0, 0, 0.6)',
  }));

  function changeProjectTitle(e){
    setProjectTitle(e.target.value);
  }


  return (
    <PopupState variant="popover" popupId="demo-popup-menu">
      {(popupState) => (
        <React.Fragment>
          <div className='menu'>
            <Button variant="contained" {...bindTrigger(popupState)} class="material-symbols-outlined">
              menu
            </Button>
            <input type='text' value={projectTitle}id='projectTitleInput' onChange={changeProjectTitle}/>
          </div>
          <StyledMenu {...bindMenu(popupState)}>
            <StyledMenuItem onClick={popupState.close}>Profile</StyledMenuItem>
            <StyledMenuItem onClick={popupState.close}>My account</StyledMenuItem>
            <StyledMenuItem onClick={popupState.close}>Logout</StyledMenuItem>
            <StyledMenuItem onClick={() => {
              openProjectManager();
              popupState.close();
              }
            }>Project Manager</StyledMenuItem>
          </StyledMenu>
        </React.Fragment>
      )}
    </PopupState>
  );
}

           
           
