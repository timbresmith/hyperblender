
/*  font.c      MIXED MODEL
 * 
 *  maart 95
 *  
 * 
 * $Id$
 *
 * ***** BEGIN GPL/BL DUAL LICENSE BLOCK *****
 *
 * The contents of this file may be used under the terms of either the GNU
 * General Public License Version 2 or later (the "GPL", see
 * http://www.gnu.org/licenses/gpl.html ), or the Blender License 1.0 or
 * later (the "BL", see http://www.blender.org/BL/ ) which has to be
 * bought from the Blender Foundation to become active, in which case the
 * above mentioned GPL option does not apply.
 *
 * The Original Code is Copyright (C) 2002 by NaN Holding BV.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): none yet.
 *
 * ***** END GPL/BL DUAL LICENSE BLOCK *****
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#ifdef WIN32
#include "BLI_winstuff.h"
#endif
#include "MEM_guardedalloc.h"

#include "BLI_arithb.h"
#include "BLI_blenlib.h"
#include "BLI_vfontdata.h"

#include "DNA_packedFile_types.h"
#include "DNA_curve_types.h"
#include "DNA_mesh_types.h"
#include "DNA_object_types.h"
#include "DNA_view3d_types.h"
#include "DNA_vfont_types.h"
#include "DNA_scene_types.h"

#include "BKE_utildefines.h"
#include "BKE_bad_level_calls.h"

#include "BKE_packedFile.h"

#include "BKE_library.h"
#include "BKE_font.h"
#include "BKE_global.h"
#include "BKE_main.h"
#include "BKE_screen.h"
#include "BKE_anim.h"
#include "BKE_curve.h"
#include "BKE_displist.h"

/* Nieuwe opzet voor vectorfont:
 *
 * geen PDrawfont meer, alles meteen naar Nurb en BezTriple
 *
 */

struct chartrans {
	float xof, yof;
	float rot;
	short linenr,charnr;
};

void free_vfont(struct VFont *vf)
{
	int i;

	if (vf == 0) return;

	if (vf->data) {
		for (i = 0; i < MAX_VF_CHARS; i++){
			while (vf->data->nurbsbase[i].first) {
				Nurb *nu = vf->data->nurbsbase[i].first;
				if (nu->bezt) MEM_freeN(nu->bezt);
				BLI_freelinkN(&vf->data->nurbsbase[i], nu);
			}
		}

		MEM_freeN(vf->data);
		vf->data = NULL;
	}
	
	if (vf->packedfile) {
		freePackedFile(vf->packedfile);
		vf->packedfile = NULL;
	}
}

static void *builtin_font_data= NULL;
static int builtin_font_size= 0;

void BKE_font_register_builtin(void *mem, int size)
{
	builtin_font_data= mem;
	builtin_font_size= size;
}

static PackedFile *get_builtin_packedfile(void)
{
	if (!builtin_font_data) {
		printf("Internal error, builtin font not loaded");

		return NULL;
	} else {
		void *mem= MEM_mallocN(builtin_font_size, "vfd_builtin");

		memcpy(mem, builtin_font_data, builtin_font_size);
	
		return newPackedFileMemory(mem, builtin_font_size);
	}
}

static VFontData *vfont_get_data(VFont *vfont)
{
	if (!vfont->data) {
		PackedFile *pf;
		
		if (BLI_streq(vfont->name, "<builtin>")) {
			pf= get_builtin_packedfile();
		} else {
			if (vfont->packedfile) {
				pf= vfont->packedfile;
			} else {
				pf= newPackedFile(vfont->name);
			}
		}
		
		if (pf) {
			vfont->data= BLI_vfontdata_from_psfont(pf);
		
			if (pf != vfont->packedfile) {
				freePackedFile(pf);
			}
		}
	}
	
	return vfont->data;	
}

VFont *load_vfont(char *name)
{
	char filename[FILE_MAXFILE];
	VFont *vfont= NULL;
	PackedFile *pf;
	int is_builtin;
	
	if (BLI_streq(name, "<builtin>")) {
		strcpy(filename, name);
		
		pf= get_builtin_packedfile();
		is_builtin= 1;
	} else {
		char dir[FILE_MAXDIR];
		
		strcpy(dir, name);
		BLI_splitdirstring(dir, filename);

		pf= newPackedFile(name);
		is_builtin= 0;
	}

	if (pf) {
		VFontData *vfd;
		
		waitcursor(1);

		vfd= BLI_vfontdata_from_psfont(pf);
		
		if (vfd) {
			vfont = alloc_libblock(&G.main->vfont, ID_VF, filename);
			vfont->data = vfd;
			
			BLI_strncpy(vfont->name, name, sizeof(vfont->name));

			// if autopack is on store the packedfile in de font structure
			if (!is_builtin && (G.fileflags & G_AUTOPACK)) {
				vfont->packedfile = pf;
			}
		}
		if (!vfont || vfont->packedfile != pf) {
			freePackedFile(pf);
		}
	
		waitcursor(0);
	}
	
	return vfont;
}

static void buildchar(Curve *cu, unsigned char ascii, float ofsx, float ofsy, float rot)
{
	BezTriple *bezt1, *bezt2;
	Nurb *nu1, *nu2;
	float *fp, fsize, shear, x, si, co;
	VFontData *vfd;
	int i;

	vfd= vfont_get_data(cu->vfont);	
	if (!vfd) return;
	
	/* maak een kopie op afstand ofsx, ofsy met shear*/
	fsize= cu->fsize;
	shear= cu->shear;
	si= (float)sin(rot);
	co= (float)cos(rot);

	nu1 = vfd->nurbsbase[ascii].first;
	while(nu1)
	{
		bezt1 = nu1->bezt;
		if (bezt1){
			nu2 =(Nurb*) MEM_mallocN(sizeof(Nurb),"duplichar_nurb");
			if (nu2 == 0) break;
			memcpy(nu2, nu1, sizeof(struct Nurb));
			nu2->resolu= cu->resolu;
			nu2->bp = 0;
			nu2->knotsu = nu2->knotsv = 0;
			nu2->flag= ME_SMOOTH;
			/* nu2->trim.first = 0; */
			/* nu2->trim.last = 0; */
			i = nu2->pntsu;

			bezt2 = (BezTriple*)MEM_mallocN(i * sizeof(BezTriple),"duplichar_bezt2"); 
			if (bezt2 == 0){
				MEM_freeN(nu2);
				break;
			}
			memcpy(bezt2, bezt1, i * sizeof(struct BezTriple));
			nu2->bezt = bezt2;
			
			if (shear != 0.0) {
				bezt2 = nu2->bezt;
				
				for (i= nu2->pntsu; i > 0; i--) {
					bezt2->vec[0][0] += shear * bezt2->vec[0][1];
					bezt2->vec[1][0] += shear * bezt2->vec[1][1];
					bezt2->vec[2][0] += shear * bezt2->vec[2][1];
					bezt2++;
				}
			}
			if(rot!=0.0) {
				bezt2= nu2->bezt;
				for (i=nu2->pntsu; i > 0; i--) {
					fp= bezt2->vec[0];
					
					x= fp[0];
					fp[0]= co*x + si*fp[1];
					fp[1]= -si*x + co*fp[1];
					x= fp[3];
					fp[3]= co*x + si*fp[4];
					fp[4]= -si*x + co*fp[4];
					x= fp[6];
					fp[6]= co*x + si*fp[7];
					fp[7]= -si*x + co*fp[7];

					bezt2++;
				}
			}
			bezt2 = nu2->bezt;
			
			for (i= nu2->pntsu; i > 0; i--) {
				fp= bezt2->vec[0];

				fp[0]= (fp[0]+ofsx)*fsize;
				fp[1]= (fp[1]+ofsy)*fsize;
				fp[3]= (fp[3]+ofsx)*fsize;
				fp[4]= (fp[4]+ofsy)*fsize;
				fp[6]= (fp[6]+ofsx)*fsize;
				fp[7]= (fp[7]+ofsy)*fsize;
				bezt2++;
			}
			
			BLI_addtail(&(cu->nurb), nu2);
		}
		nu1 = nu1->next;
	}
}


struct chartrans *text_to_curve(Object *ob, int mode) 
{
	VFont *vfont;
	VFontData *vfd;
	Curve *cu, *cucu;
	struct chartrans *chartransdata, *ct;
	float distfac, tabfac, ctime, dtime, tvec[4], vec[4], rotvec[3], minx, maxx, miny, maxy;
	float cmat[3][3], timeofs, si, co, sizefac;
	float *f, maxlen=0, xof, yof, xtrax, linedist, *linedata, *linedata2;
	int i, slen, oldflag;
	short cnr=0, lnr=0;
	char ascii, *mem;

	/* opmerking: berekeningen altijd tot en met de '\0' van de string omdat
	   de cursor op die plek moet kunnen staan */

	if(ob->type!=OB_FONT) return 0;

	cu= ob->data;

	vfont= cu->vfont;
	if (vfont==0) return 0;
	if (cu->str==0) return 0;

	vfd= vfont_get_data(vfont);
	if (!vfd) return 0;
	
	/* aantal regels tellen */
	mem= cu->str;
	slen = strlen(mem);
	cu->lines= 1;
	for (i= 0; i<=slen; i++, mem++) {
		ascii = *mem;
		if(ascii== '\n' || ascii== '\r') cu->lines++;
	}

	/* bereken ofset en rotatie van iedere letter */
	ct = chartransdata =
		(struct chartrans*)MEM_callocN((slen+1)* sizeof(struct chartrans),"buildtext");
	linedata= MEM_mallocN(sizeof(float)*cu->lines,"buildtext2");
	linedata2= MEM_mallocN(sizeof(float)*cu->lines,"buildtext2");
	xof= cu->xof;
	yof= cu->yof;

	xtrax= 0.5f*cu->spacing-0.5f;
	linedist= cu->linedist;

	for (i = 0 ; i<=slen ; i++) {
		ascii = cu->str[i];
		if(ascii== '\n' || ascii== '\r' || ascii==0) {
			ct->xof= xof;
			ct->yof= yof;
			ct->linenr= lnr;
			ct->charnr= cnr;
			
			/* alleen lege regels mogen kleiner dan 1 zijn */
			if( linedist<1.0) {
				if(i<slen && (cu->str[i+1]=='\r' || cu->str[i+1]=='\n')) yof-= linedist;
				else yof-= 1.0;
			}
			else yof-= linedist;
			
			maxlen= MAX2(maxlen, xof);
			linedata[lnr]= xof;
			linedata2[lnr]= cnr;
			xof= cu->xof;
			lnr++;
			cnr= 0;
		}
		else if(ascii==9) {	/* TAB */
			ct->xof= xof;
			ct->yof= yof;
			ct->linenr= lnr;
			ct->charnr= cnr++;

			tabfac= (xof-cu->xof+0.01f);
			tabfac= (float)(2.0*ceil(tabfac/2.0));
			xof= cu->xof+tabfac;
		}
		else {
			ct->xof= xof;
			ct->yof= yof;
			ct->linenr= lnr;
			ct->charnr= cnr++;

			xof += vfd->width[ascii] + xtrax;
		}
		ct++;
	}

	/* met alle fontsettings plekken letters berekenen */
	if(cu->spacemode!=CU_LEFT && lnr>1) {
		ct= chartransdata;

		if(cu->spacemode==CU_RIGHT) {
			for(i=0;i<lnr;i++) linedata[i]= maxlen-linedata[i];
			for (i=0; i<=slen; i++) {
				ct->xof+= linedata[ct->linenr];
				ct++;
			}
		} else if(cu->spacemode==CU_MIDDLE) {
			for(i=0;i<lnr;i++) linedata[i]= (maxlen-linedata[i])/2;
			for (i=0; i<=slen; i++) {
				ct->xof+= linedata[ct->linenr];
				ct++;
			}
		} else if(cu->spacemode==CU_FLUSH) {
			for(i=0;i<lnr;i++)
				if(linedata2[i]>1)
					linedata[i]= (maxlen-linedata[i])/(linedata2[i]-1);
			for (i=0; i<=slen; i++) {
				ct->xof+= ct->charnr*linedata[ct->linenr];
				ct++;
			}
		}
	}
	
	/* TEXT ON CURVE */
	if(cu->textoncurve) {
		cucu= cu->textoncurve->data;
		
		oldflag= cucu->flag;
		cucu->flag |= (CU_PATH+CU_FOLLOW);
		
		if(cucu->path==0) calc_curvepath(cu->textoncurve);
		if(cucu->path) {
			

			Mat3CpyMat4(cmat, cu->textoncurve->obmat);
			sizefac= Normalise(cmat[0])/cu->fsize;
			
			minx=miny= 1.0e20f;
			maxx=maxy= -1.0e20f;
			ct= chartransdata;
			for (i=0; i<=slen; i++, ct++) {
				if(minx>ct->xof) minx= ct->xof;
				if(maxx<ct->xof) maxx= ct->xof;
				if(miny>ct->yof) miny= ct->yof;
				if(maxy<ct->yof) maxy= ct->yof;
			}
			
			/* we zetten de x-coordinaat exact op de curve, de y wordt geroteerd */
			
			/* de lengte correctie */
			distfac= sizefac*cucu->path->totdist/(maxx-minx);
			timeofs= 0.0;
			
			if(distfac > 1.0) {
				/* pad langer dan tekst: spacemode doet mee */
				distfac= 1.0f/distfac;
				
				if(cu->spacemode==CU_RIGHT) {
					timeofs= 1.0f-distfac;
				}
				else if(cu->spacemode==CU_MIDDLE) {
					timeofs= (1.0f-distfac)/2.0f;
				}
				else if(cu->spacemode==CU_FLUSH) distfac= 1.0f;
				
			}
			else distfac= 1.0;
			
			distfac/= (maxx-minx);
			
			timeofs+= distfac*cu->xof;	/* niet cyclic */
			
			ct= chartransdata;
			for (i=0; i<=slen; i++, ct++) {
				
				/* roteren rond centrum letter */
				ascii = cu->str[i];
				dtime= distfac*0.35f*vfd->width[ascii];	/* Waarom is 0.5 te groot? */
				dtime= distfac*0.0f*vfd->width[ascii];	/* Waarom is 0.5 te groot? */
				
				ctime= timeofs + distfac*( ct->xof - minx);
				CLAMP(ctime, 0.0, 1.0);
				
				/* de goede plek EN de goede rotatie apart berekenen */
				where_on_path(cu->textoncurve, ctime, vec, tvec);
				where_on_path(cu->textoncurve, ctime+dtime, tvec, rotvec);
				
				VecMulf(vec, sizefac);
				
				ct->rot= (float)(M_PI-atan2(rotvec[1], rotvec[0]));

				si= (float)sin(ct->rot);
				co= (float)cos(ct->rot);

				yof= ct->yof;
				
				ct->xof= vec[0] + si*yof;
				ct->yof= vec[1] + co*yof;
				
			}
			cucu->flag= oldflag;
		}
	}


	if(mode==FO_CURSUP || mode==FO_CURSDOWN) {
		/* 2: curs omhoog
		   3: curs omlaag */
		ct= chartransdata+cu->pos;
		
		if(mode==FO_CURSUP && ct->linenr==0);
		else if(mode==FO_CURSDOWN && ct->linenr==lnr);
		else {
			if(mode==FO_CURSUP) lnr= ct->linenr-1;
			else lnr= ct->linenr+1;
			cnr= ct->charnr;
			/* zoek karakter met lnr en cnr */
			cu->pos= 0;
			ct= chartransdata;
			for (i= 0; i<slen; i++) {
				if(ct->linenr==lnr) {
					if(ct->charnr==cnr) break;
					if( (ct+1)->charnr==0) break;
				}
				else if(ct->linenr>lnr) break;
				cu->pos++;
				ct++;
			}
		}
	}
	
	/* eerst cursor */
	if(ob==G.obedit) {
		ct= chartransdata+cu->pos;
		si= (float)sin(ct->rot);
		co= (float)cos(ct->rot);
				
		f= G.textcurs[0];
		
		f[0]= cu->fsize*(-0.1f*co + ct->xof);
		f[1]= cu->fsize*(0.1f*si + ct->yof);
		
		f[2]= cu->fsize*(0.1f*co + ct->xof);
		f[3]= cu->fsize*(-0.1f*si + ct->yof);
		
		f[4]= cu->fsize*( 0.1f*co + 0.8f*si + ct->xof);
		f[5]= cu->fsize*(-0.1f*si + 0.8f*co + ct->yof);
		
		f[6]= cu->fsize*(-0.1f*co + 0.8f*si + ct->xof);
		f[7]= cu->fsize*( 0.1f*si + 0.8f*co + ct->yof);
		
	}

	if(mode==0) {
		/* nurbdata maken */
		
		freeNurblist(&cu->nurb);
		
		ct= chartransdata;
		for (i= 0; i<slen; i++) {
			ascii = cu->str[i];
			buildchar(cu, ascii, ct->xof, ct->yof, ct->rot);
			ct++;
		}
	}

	MEM_freeN(linedata);
	MEM_freeN(linedata2);

	if(mode==FO_DUPLI) {
		return chartransdata;
	}

	MEM_freeN(chartransdata);
	return 0;
}


/* ***************** DUPLI  ***************** */

static Object *find_family_object(Object **obar, char *family, char ch)
{
	Object *ob;
	int flen;
	
	if( obar[ch] ) return obar[ch];
	
	flen= strlen(family);
	
	ob= G.main->object.first;
	while(ob) {
		if( ob->id.name[flen+2]==ch ) {
			if( strncmp(ob->id.name+2, family, flen)==0 ) break;
		}
		ob= ob->id.next;
	}
	
	obar[ch]= ob;
	
	return ob;
}


void font_duplilist(Object *par)
{
	extern ListBase duplilist;
	Object *ob, *newob, *obar[256];
	Curve *cu;
	struct chartrans *ct, *chartransdata;
	float vec[3], pmat[4][4], fsize, xof, yof;
	int slen, a;
	
	Mat4CpyMat4(pmat, par->obmat);

	/* in par staat een familienaam, deze gebruiken om objecten te vinden */

	chartransdata= text_to_curve(par, FO_DUPLI);
	if(chartransdata==0) return;
	
	memset(obar, 0, 256*4);
	
	cu= par->data;
	slen= strlen(cu->str);
	fsize= cu->fsize;
	xof= cu->xof;
	yof= cu->yof;
	
	ct= chartransdata;
	set_displist_onlyzero(1);

	for(a=0; a<slen; a++, ct++) {
	
		ob= find_family_object(obar, cu->family, cu->str[a]);
		if(ob) {
			
			makeDispList(ob);
			
			vec[0]= fsize*(ct->xof - xof);
			vec[1]= fsize*(ct->yof - yof);
			vec[2]= 0.0;
	
			Mat4MulVecfl(pmat, vec);
			
			newob= MEM_mallocN(sizeof(Object), "newobj dupli");
			memcpy(newob, ob, sizeof(Object));
			newob->flag |= OB_FROMDUPLI;
			newob->id.newid= (ID *)par;		/* duplicator bewaren */
			newob->totcol= par->totcol;	/* voor give_current_material */
			
			Mat4CpyMat4(newob->obmat, par->obmat);
			VECCOPY(newob->obmat[3], vec);
			
			newob->parent= 0;
			newob->track= 0;
			
			BLI_addtail(&duplilist, newob);
		}
		
	}
	set_displist_onlyzero(0);
	MEM_freeN(chartransdata);
}
